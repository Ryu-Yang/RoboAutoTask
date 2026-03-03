import math
import numpy as np
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from sensor_msgs.msg import JointState

from roboautotask.configs import topic
from roboautotask.configs import robot

from roboautotask.utils.math import quaternion_slerp
from roboautotask.utils.pose import save_pose_to_file


# ================= 主驱动类 =================
class InterpolationDriver(Node):
    def __init__(self, start_pos, start_quat, end_pos, end_quat, steps=50, gripper_pos=100.0):
        super().__init__('RoboAutoTask_Interpolation_Driver')
        
        # --- Publishers ---
        self.left_ee_publisher = self.create_publisher(PoseStamped, topic.LEFT_EE_PUB, 10)
        self.right_ee_publisher = self.create_publisher(PoseStamped, topic.RIGHT_EE_PUB, 10)
        self.left_gripper_pub = self.create_publisher(JointState, topic.LEFT_GRIPPER_PUB, 10)
        self.right_gripper_pub = self.create_publisher(JointState, topic.RIGHT_GRIPPER_PUB, 10)

        # --- Subscriptions ---
        self.left_ee_subscription = self.create_subscription(
            PoseStamped, topic.LEFT_EE_SUB, self.left_ee_sub_callback, 10
        )
        self.left_gripper_subscription = self.create_subscription(
            JointState, topic.LEFT_GRIPPER_SUB, self.left_gripper_sub_callback, 10
        )
        
        # --- 轨迹生成 ---
        self.trajectory = []
        p_start = np.array(start_pos)
        p_end = np.array(end_pos)
        q_start = np.array(start_quat)
        q_end = np.array(end_quat)

        # 抬升参数
        lift_height = 0.07  # 最大抬升高度

        for i in range(steps + 1):
            t = i / float(steps)
            pos = (1 - t) * p_start + t * p_end

            # 抛物线抬升：Δz = h * (1 - (2t - 1)^2)
            dz = lift_height * (1.0 - (2.0 * t - 1.0) ** 2)
            pos[2] += dz  # 修改 Z 轴

            quat = quaternion_slerp(q_start, q_end, t)
            self.trajectory.append((pos, quat))
            
        self.current_idx = 0
        
        # --- 状态控制 ---
        # 状态枚举: 'MOVING' -> 'STABILIZING' -> 'GRIPPING' -> 'DONE'
        self.state = 'MOVING'  
        self.target_gripper_pos = float(gripper_pos)
        self.current_gripper_val = None # 存储读取到的当前夹爪值
        
        self.current_ee_pose = None # 存储读取到的当前EE位姿

        # 计时器用于夹爪动作的延时等待
        self.grip_wait_start = None 
        self.GRIP_DURATION = 1.0  # 秒，给予夹爪物理运动的时间

        # 10Hz 控制循环
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"Task Started. Steps: {steps}, Target Gripper: {gripper_pos}")


        # ---------- 超时控制(10s未执行完操作说明卡住) ---------
        self.start_time = time.time()
        self.TIMEOUT_LIMIT = 10.0  # 10秒超时
        self.success = False      # 记录最终是否成功执行

    def left_ee_sub_callback(self, msg):
        self.current_ee_pose = msg.pose.position

    def left_gripper_sub_callback(self, msg):
        # 假设 msg.position 是一个列表，取第一个值
        if len(msg.position) > 0:
            self.current_gripper_val = msg.position[0]

    def _publish_static_right_arm(self):
        """ 始终发布右臂的固定姿态 """
        msg = PoseStamped()
        msg.header.frame_id = ''
        msg.header.stamp = self.get_clock().now().to_msg()
        # 固定坐标
        msg.pose.position = Point(x=0.05879554525706433, y=-0.3371213341001924, z=0.33776230038533134)
        msg.pose.orientation = Quaternion(x=0.010879876331725758, y=-0.010736008971160126, z=-0.009139838757527292, w=0.9998452687588322)
        self.right_ee_publisher.publish(msg)
        
        # 右臂夹爪也保持固定 (例如 100)
        g_msg = JointState()
        g_msg.header.stamp = self.get_clock().now().to_msg()
        g_msg.position = [100.0]
        self.right_gripper_pub.publish(g_msg)

    def timer_callback(self):
        # 检查是否超时，超时直接退出
        elapsed_total = time.time() - self.start_time
        if elapsed_total > self.TIMEOUT_LIMIT:
            self.get_logger().error(f"Motion Timeout! Elapsed: {elapsed_total:.2f}s")
            self.success = False
            raise SystemExit # 触发退出逻辑
        
        # 1. 始终发布右臂（守护进程）
        self._publish_static_right_arm()

        # 2. 准备左臂消息
        msg_ee = PoseStamped()
        msg_ee.header.frame_id = ''
        msg_ee.header.stamp = self.get_clock().now().to_msg()

        msg_grip = JointState()
        msg_grip.header.stamp = self.get_clock().now().to_msg()

        # ============ 状态机逻辑 ============
        
        # --- 状态 1: 机械臂运动中 ---
        if self.state == 'MOVING':
            if self.current_idx < len(self.trajectory):
                # 发送轨迹点
                pos, quat = self.trajectory[self.current_idx]
                msg_ee.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
                msg_ee.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                self.left_ee_publisher.publish(msg_ee)
                self.current_idx += 1

                # 夹爪行为：保持当前实际位置 (防止松脱)
                if self.current_gripper_val is not None:
                    msg_grip.position = [float(self.current_gripper_val)]
                    self.left_gripper_pub.publish(msg_grip)
            else:
                # 轨迹发完，进入稳定检测
                self.state = 'STABILIZING'
                self.get_logger().info("Trajectory finished. Waiting for arm to stabilize...")

        # --- 状态 2: 机械臂到位确认 ---
        elif self.state == 'STABILIZING':
            # 持续发布最后一个点，维持力矩
            pos, quat = self.trajectory[-1]
            msg_ee.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            msg_ee.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.left_ee_publisher.publish(msg_ee)
            
            # 夹爪继续保持
            if self.current_gripper_val is not None:
                msg_grip.position = [float(self.current_gripper_val)]
                self.left_gripper_pub.publish(msg_grip)

            # 检查误差
            if self.current_ee_pose is not None:
                curr = self.current_ee_pose
                dist = math.sqrt((curr.x - pos[0])**2 + (curr.y - pos[1])**2 + (curr.z - pos[2])**2)
                if dist < 0.02: # 2cm 误差范围内认为到位
                    self.state = 'GRIPPING'
                    self.grip_wait_start = time.time()
                    self.get_logger().info(f"Arm Stabilized (Err: {dist:.3f}). Actuating Gripper to {self.target_gripper_pos}...")

        # --- 状态 3: 执行夹爪动作 ---
        elif self.state == 'GRIPPING':
            # 机械臂继续维持最后位置
            pos, quat = self.trajectory[-1]
            msg_ee.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            msg_ee.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.left_ee_publisher.publish(msg_ee)

            # 发送新的目标夹爪位置
            msg_grip.position = [self.target_gripper_pos]
            self.left_gripper_pub.publish(msg_grip)

            # 检查是否完成 (时间延迟 + 误差判断)
            elapsed = time.time() - self.grip_wait_start
            
            # 判断逻辑：时间超过1秒 且 (夹爪反馈接近目标 或 只是单纯等待时间)
            # 这里为了简单稳健，使用单纯的时间等待，或者你可以加上 abs(self.current_gripper_val - target) < 5
            is_physically_reached = False
            if self.current_gripper_val is not None:
                if abs(self.current_gripper_val - self.target_gripper_pos) < 5.0:
                    is_physically_reached = True
            
            # 至少等待0.5秒，如果物理到位了或者等待超过1.5秒强制结束
            if (elapsed > 0.3 and is_physically_reached) or (elapsed > 1):
                self.state = 'DONE'
                self.get_logger().info("Gripper action completed.")

        # --- 状态 4: 结束 ---
        elif self.state == 'DONE':
            self.success = True
            raise SystemExit


class getCurrPoseDriver(Node):
    def __init__(self):
        super().__init__('RoboAutoTask_getCurrPose_Driver')

        # ---create subscriber--
        self.left_ee_subscription = self.create_subscription(
            PoseStamped, topic.LEFT_EE_SUB, self.left_ee_sub_callback, 10
        )
        self.left_gripper_subscription = self.create_subscription(
            JointState, topic.LEFT_GRIPPER_SUB, self.left_gripper_sub_callback, 10
        )

        # ---parameter

        self.cb_idx = 0
        self.target_count = 3  # 设定的采样次数
        self.pos_list = []  # 存储 [x, y, z]
        self.ori_list = []  # 存储 [x, y, z, w]
        self.current_gripper = 100.0

    def left_ee_sub_callback(self,msg):
        # 1. 防止空值或异常判断
        if msg.pose is None:
            # self.get_logger().warn("接收到空的 Pose 数据，跳过本次采样")
            return

        # 2. 提取数据并存入列表
        p = msg.pose.position
        o = msg.pose.orientation
        
        # 检查坐标是否包含有效数值（防止某些驱动初始化发出的全0或NaN数据）
        if not all(map(lambda v: isinstance(v, float), [p.x, p.y, p.z])):
            return

        self.pos_list.append([p.x, p.y, p.z])
        self.ori_list.append([o.x, o.y, o.z, o.w])
        
        self.cb_idx += 1
        # self.get_logger().info(f"已收集数据: {self.cb_idx}/{self.target_count}")

        # 3. 达到次数后计算平均值并退出
        if self.cb_idx >= self.target_count:
            self.calculate_and_finish()


    def left_gripper_sub_callback(self,msg):
        # 假设 msg.position 是一个列表，取第一个值
        if len(msg.position) > 0:
            self.current_gripper = msg.position[0]

    def calculate_and_finish(self):
        # 将列表转换为 NumPy 数组方便按列求平均
        avg_pos = np.mean(self.pos_list, axis=0)
        avg_ori = np.mean(self.ori_list, axis=0)
        
        # 注意：四元数简单求平均在严格数学上是不准确的（应使用SLERP或归一化），
        # 但如果姿态波动极小，简单平均+归一化是常见的工程简化方案。
        avg_ori = avg_ori / np.linalg.norm(avg_ori) 

        # self.get_logger().info(f"--- 采样完成 ---")
        # self.get_logger().info(f"平均位置: {avg_pos}")
        # self.get_logger().info(f"平均姿态: {avg_ori}")

        # 保存文件
        save_pose_to_file('latest_pose.txt', avg_pos, avg_ori)
        
        # 触发节点销毁和程序退出
        raise SystemExit




def execute_motion(start_pos, start_quat, target_pos, target_quat, gripper_pos):
    """
    外部调用接口
    """
    rclpy.init()
    
    # 创建节点并运行
    node = InterpolationDriver(start_pos, start_quat, target_pos, target_quat, steps=60, gripper_pos=gripper_pos)
    
    # 更新姿态文件
    save_pose_to_file("latest_pose.txt", target_pos, target_quat)

    motion_success = False
    try:
        rclpy.spin(node)
        motion_success = node.success # 获取节点运行后的成功状态
    except SystemExit:
        motion_success = node.success
    except Exception as e:
        print(f"Unexpected error: {e}")
        motion_success = False
    finally:
        node.destroy_node()
        # 检查是否还有其他节点在使用 rclpy，如果没有则 shutdown
        if rclpy.ok():
            rclpy.shutdown()
    # --- 超时或失败后的逻辑处理 ---
    if not motion_success:
        print("Motion failed or timed out. Triggering reset...")
        return False
    return True

def set_gripper_position(position, side='left'):
    # 因为不需要单独的类控制夹爪，目前废弃
    print("Warning: set_gripper_position is deprecated. Use execute_motion for synchronized control.")
    pass


def MeasureCurrPose():
    """
    外部调用接口
    """
    rclpy.init()
    
    # 创建节点并运行
    node = getCurrPoseDriver()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        # 检查是否还有其他节点在使用 rclpy，如果没有则 shutdown
        if rclpy.ok():
            rclpy.shutdown()