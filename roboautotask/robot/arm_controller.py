import time
import math
import numpy as np
from enum import Enum, auto

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from sensor_msgs.msg import JointState
from roboautotask.utils.math import quaternion_slerp
from roboautotask.configs.robot import CONTROL_PARAMS

class ArmState(Enum):
    IDLE = auto()        # 空闲/保持状态
    MOVING = auto()      # 轨迹运动中
    STABILIZING = auto() # 到位稳定中
    GRIPPING = auto()    # 夹爪动作中
    FINISHED = auto()    # 动作完成

class ArmController:
    def __init__(self, node, side, config):
        """
        :param node: ROS2 Node 句柄 (用于日志和时间)
        :param side: 'left' or 'right'
        :param config: 该臂的配置字典
        """
        self.node = node
        self.side = side
        self.logger = node.get_logger()
        
        # --- 通信发布者 ---
        self.ee_pub = node.create_publisher(PoseStamped, config['ee_pub'], 10)
        self.gripper_pub = node.create_publisher(JointState, config['gripper_pub'], 10)
        
        # --- 状态与数据 ---
        self.state = ArmState.IDLE
        self.current_ee_pose = None      # 传感器反馈的当前位姿 (Point)
        self.current_gripper_val = None  # 传感器反馈的当前夹爪值
        
        # --- 保持/目标位姿 ---
        # 如果配置中有 home_pose 则使用，否则初始化为 None (等待第一次 callback)
        self.hold_pose = config.get('home_pose') 
        
        # --- 运动规划数据 ---
        self.trajectory = []
        self.traj_idx = 0
        self.target_gripper = 100.0
        
        # --- 计时器 ---
        self.state_start_time = 0.0

    def update_feedback(self, ee_pos=None, gripper_val=None):
        """更新传感器反馈数据"""
        if ee_pos is not None:
            self.current_ee_pose = ee_pos
            # 如果刚启动且没有保持点，将当前位置设为保持点
            if self.hold_pose is None:
                # 简单处理：姿态默认直立或需要从 msg 获取 quaternion (这里简化处理)
                # 建议：在 update_feedback 中传入完整的 Pose 而不仅仅是 Position
                pass 

        if gripper_val is not None:
            self.current_gripper_val = gripper_val

    def set_motion(self, start_pos, start_quat, end_pos, end_quat, gripper_pos):
        """设置新的运动任务"""
        self.trajectory = []
        steps = CONTROL_PARAMS['steps']
        lift_height = CONTROL_PARAMS['lift_height']
        
        p_start = np.array(start_pos)
        p_end = np.array(end_pos)
        q_start = np.array(start_quat)
        q_end = np.array(end_quat)

        # 生成轨迹
        for i in range(steps + 1):
            t = i / float(steps)
            pos = (1 - t) * p_start + t * p_end
            
            # 抛物线抬升
            if lift_height > 0:
                dz = lift_height * (1.0 - (2.0 * t - 1.0) ** 2)
                pos[2] += dz

            quat = quaternion_slerp(q_start, q_end, t)
            self.trajectory.append((pos, quat))
        
        self.traj_idx = 0
        self.target_gripper = float(gripper_pos)
        self.state = ArmState.MOVING
        self.logger.info(f"[{self.side.upper()}] Motion planned. Steps: {steps}")

    def tick(self):
        """
        核心状态机循环，每次 timer callback 调用一次。
        返回: is_active (bool) - 该臂是否正在执行任务
        """
        now = time.time()
        
        # 准备发布的消息容器
        msg_ee = PoseStamped()
        msg_ee.header.frame_id = '' # 根据实际情况填
        msg_ee.header.stamp = self.node.get_clock().now().to_msg()
        
        msg_grip = JointState()
        msg_grip.header.stamp = self.node.get_clock().now().to_msg()

        # ================= 状态机 =================
        
        # --- 1. 空闲/保持状态 ---
        if self.state == ArmState.IDLE or self.state == ArmState.FINISHED:
            if self.hold_pose:
                p = self.hold_pose['pos']
                q = self.hold_pose['quat']
                # 保持位置，夹爪也保持(如果有记录)
                g = self.hold_pose.get('gripper', 100.0) 
                self._fill_and_pub(msg_ee, msg_grip, p, q, g)
            return False # Not active

        # --- 2. 运动状态 ---
        elif self.state == ArmState.MOVING:
            if self.traj_idx < len(self.trajectory):
                pos, quat = self.trajectory[self.traj_idx]
                self.traj_idx += 1
                
                # 运动中，夹爪保持当前读数 (防止掉落)
                g_val = self.current_gripper_val if self.current_gripper_val else 100.0
                
                self._fill_and_pub(msg_ee, msg_grip, pos, quat, g_val)
                
                # 更新保持点，万一中断可以停在这里
                self.hold_pose = {'pos': pos, 'quat': quat, 'gripper': g_val}
            else:
                # 轨迹发完，进入稳定检测
                self.state = ArmState.STABILIZING
                self.state_start_time = now
                self.logger.info(f"[{self.side.upper()}] Trajectory done. Stabilizing...")

        # --- 3. 稳定状态 ---
        elif self.state == ArmState.STABILIZING:
            # 持续发布最后一个点
            pos, quat = self.trajectory[-1]
            g_val = self.current_gripper_val if self.current_gripper_val else 100.0
            self._fill_and_pub(msg_ee, msg_grip, pos, quat, g_val)
            
            # 计算误差
            dist = 999.0
            if self.current_ee_pose:
                curr = self.current_ee_pose
                dist = math.sqrt((curr.x - pos[0])**2 + (curr.y - pos[1])**2 + (curr.z - pos[2])**2)
            
            # 判断逻辑：误差小 OR 超时
            is_stable = dist < CONTROL_PARAMS['stabilize_threshold']
            is_timeout = (now - self.state_start_time) > CONTROL_PARAMS['stabilize_timeout']
            
            if is_stable or is_timeout:
                if is_timeout:
                    self.logger.warn(f"[{self.side.upper()}] Stabilize timeout (Err: {dist:.3f}). Forcing continue.")
                
                self.state = ArmState.GRIPPING
                self.state_start_time = now
                self.logger.info(f"[{self.side.upper()}] Gripping to {self.target_gripper}...")

        # --- 4. 夹爪动作 ---
        elif self.state == ArmState.GRIPPING:
            # 机械臂不动
            pos, quat = self.trajectory[-1]
            # 夹爪动作
            self._fill_and_pub(msg_ee, msg_grip, pos, quat, self.target_gripper)
            
            # 简单的时间等待逻辑
            if (now - self.state_start_time) > CONTROL_PARAMS['grip_duration']:
                self.state = ArmState.FINISHED
                # 更新最终保持状态
                self.hold_pose = {'pos': pos, 'quat': quat, 'gripper': self.target_gripper}
                self.logger.info(f"[{self.side.upper()}] Action Finished.")

        return True # Active

    def _fill_and_pub(self, msg_ee, msg_grip, pos, quat, gripper_val):
        """辅助函数：填充消息并发布"""
        msg_ee.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        msg_ee.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        msg_grip.position = [float(gripper_val)]
        
        self.ee_pub.publish(msg_ee)
        self.gripper_pub.publish(msg_grip)