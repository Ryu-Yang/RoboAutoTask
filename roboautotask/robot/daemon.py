import math
import numpy as np
import time

from robodriver.robots.utils import make_robot_from_config, Robot

from roboautotask.configs import topic
from roboautotask.configs import robot

from roboautotask.robot.utils import get_pose_from_observation, create_action
from roboautotask.utils.math import quaternion_slerp
from roboautotask.utils.pose import save_pose_to_file


# ================= 主驱动类 =================
class Daemon:
    def __init__(self, robot: Robot, use_arm: str = "left"):
        self.robot = robot
        self.use_arm = use_arm

        # --- 状态控制 ---
        # 状态枚举: 'MOVING' -> 'STABILIZING' -> 'GRIPPING' -> 'DONE'
        self.state = 'MOVING'
        
        self.current_gripper_val = None
        self.current_ee_pose = None 

        self.target_gripper_pos = None


        # 计时器用于夹爪动作的延时等待
        self.grip_wait_start = None 


        # ---------- 超时控制(10s未执行完操作说明卡住) ---------
        self.start_time = time.time()
        self.TIMEOUT_LIMIT = 10.0  # 10秒超时
        self.success = False      # 记录最终是否成功执行

        pass
        
    def execute_motion(self, target_pos, target_quat, steps=60, gripper_pos=100):
        #从obs中获取从臂当前pose
        obs = self.robot.get_observation()
        start_pos, start_quat =  get_pose_from_observation(obs, self.use_arm)

        # --- 轨迹生成 ---
        self.trajectory = []
        p_start = np.array(start_pos)
        p_end = np.array(target_pos)
        q_start = np.array(start_quat)
        q_end = np.array(target_quat)

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
            
        while True:
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

                    self.robot.send_action(create_action(0))
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
        
        
        return True
        