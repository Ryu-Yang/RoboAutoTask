import math
import numpy as np
import time
import logging_mp
import threading
from typing import Any, Dict, Optional, Union

from robodriver.robots.utils import make_robot_from_config, Robot

from roboautotask.configs import topic
from roboautotask.configs import robot

from roboautotask.robot.utils import get_pose_from_observation, get_gripper_from_observation, create_action
from roboautotask.utils.math import quaternion_slerp
from roboautotask.utils.pose import save_pose_to_file


logger = logging_mp.get_logger(__name__)


# ================= 主驱动类 =================
class Daemon:
    def __init__(self, robot: Robot, use_arm: str = "left", unuse_arm: str = "right"):
        self.robot = robot
        self.use_arm = use_arm
        self.unuse_arm = unuse_arm

        self.state = 'MOVING'  # 状态控制。状态枚举: 'MOVING' -> 'STABILIZING' -> 'GRIPPING' -> 'DONE'
        self.grip_wait_start = None  # 计时器用于夹爪动作的延时等待

        self.timeout = 20.0  # 超时控制(10s未执行完操作说明卡住)

        self.fps = 10

        self.running = True

        self.data_lock = threading.Lock()
        self.pre_action: Union[Any, Dict[str, Any]] = None
        self.obs_action: Union[Any, Dict[str, Any]] = None
        self.observation: Union[Any, Dict[str, Any]] = None



    @property
    def cameras_info(self):
        cameras = {}
        for name, camera in self.robot.cameras.items():
            if hasattr(camera, "camera_index"):
                cameras[name] = camera.camera_index
            elif hasattr(camera, "index_or_path"):
                cameras[name] = camera.index_or_path
        return cameras
    
    def start(self):
        if not self.robot.is_connected:
            self.robot.connect()
        logger.info("Connect robot success")

        obs = self.robot.get_observation()

        obs_use_gripper = get_gripper_from_observation(obs, self.use_arm)
        obs_use_pos, obs_use_quat = get_pose_from_observation(obs, self.use_arm)

        obs_unuse_gripper = get_gripper_from_observation(obs, self.unuse_arm)
        obs_unuse_pos, obs_unuse_quat = get_pose_from_observation(obs, self.unuse_arm)

        _use_action = create_action(obs_use_pos, obs_use_quat, obs_use_gripper, use_arm=self.use_arm)
        _unuse_action = create_action(obs_unuse_pos, obs_unuse_quat, obs_unuse_gripper, use_arm=self.unuse_arm)

        self.obs_action = {**_use_action, **_unuse_action}

    def stop(self):
        pass

    def execute_motion(self, target_pos, target_quat, steps=60, gripper_pos=100):
        start_time = time.time()
        current_idx = 0

        #从obs中获取从臂当前pose
        obs = self.robot.get_observation()
        start_pos, start_quat = get_pose_from_observation(obs, self.use_arm)    

        # --- 轨迹生成 ---
        trajectory = []
        p_start = np.array(start_pos)
        p_end = np.array(target_pos)
        q_start = np.array(start_quat)
        q_end = np.array(target_quat)

        # 抬升参数
        lift_height = 0.07

        for i in range(int(steps) + 1):
            t = i / float(steps)
            pos = (1 - t) * p_start + t * p_end

            # 抛物线抬升：Δz = h * (1 - (2t - 1)^2)
            dz = lift_height * (1.0 - (2.0 * t - 1.0) ** 2)
            pos[2] += dz  # 修改 Z 轴

            quat = quaternion_slerp(q_start, q_end, t)
            trajectory.append((pos, quat))
            
        while True:
            fps_start_time = time.perf_counter()

            obs = self.robot.get_observation()

            obs_use_gripper = get_gripper_from_observation(obs, self.use_arm)
            obs_use_pos, obs_use_quat = get_pose_from_observation(obs, self.use_arm)

            obs_unuse_gripper = get_gripper_from_observation(obs, self.unuse_arm)
            obs_unuse_pos, obs_unuse_quat = get_pose_from_observation(obs, self.unuse_arm)

            if self.state == 'MOVING':
                if current_idx < len(trajectory):
                    pos, quat = trajectory[current_idx]

                    use_action = create_action(pos, quat, obs_use_gripper, use_arm=self.use_arm)
                    unuse_action = create_action(obs_unuse_pos, obs_unuse_quat, obs_unuse_gripper, use_arm=self.unuse_arm)
                    action = {**use_action, **unuse_action}

                    self.robot.send_action(action)
                    self.set_obs_action(action)
                    # logger.info(f"action: {action}")

                    current_idx += 1
                else:
                    self.state = 'STABILIZING'
                    logger.info("Trajectory finished. Waiting for arm to stabilize...")

            elif self.state == 'STABILIZING':
                pos, quat = trajectory[-1]

                use_action = create_action(pos, quat, obs_use_gripper, use_arm=self.use_arm)
                unuse_action = create_action(obs_unuse_pos, obs_unuse_quat, obs_unuse_gripper, use_arm=self.unuse_arm)
                action = {**use_action, **unuse_action}

                self.robot.send_action(action)
                self.set_obs_action(action)
                # logger.info(f"action: {action}")

                if obs_use_pos is not None:
                    curr = obs_use_pos
                    dist = math.sqrt((curr[0] - pos[0])**2 + (curr[1] - pos[1])**2 + (curr[2] - pos[2])**2)
                    if dist < 0.02: # 2cm 误差范围内认为到位
                        self.state = 'GRIPPING'
                        self.grip_wait_start = time.time()
                        logger.info(f"Arm Stabilized (Err: {dist:.3f}). Actuating Gripper to {gripper_pos}...")

            # --- 状态 3: 执行夹爪动作 ---
            elif self.state == 'GRIPPING':
                # 机械臂继续维持最后位置
                pos, quat = trajectory[-1]

                use_action = create_action(pos, quat, gripper_pos, use_arm=self.use_arm)
                unuse_action = create_action(obs_unuse_pos, obs_unuse_quat, obs_unuse_gripper, use_arm=self.unuse_arm)
                action = {**use_action, **unuse_action}

                self.robot.send_action(action)
                self.set_obs_action(action)
                # logger.info(f"action: {action}")

                # 检查是否完成 (时间延迟 + 误差判断)
                elapsed = time.time() - self.grip_wait_start
                
                # 判断逻辑：时间超过1秒 且 (夹爪反馈接近目标 或 只是单纯等待时间)
                is_physically_reached = False
                if obs_use_gripper is not None:
                    if abs(obs_use_gripper - gripper_pos) < 5.0:
                        is_physically_reached = True
                
                # 至少等待0.5秒，如果物理到位了或者等待超过1.5秒强制结束
                if (elapsed > 0.3 and is_physically_reached) or (elapsed > 1):
                    self.state = 'DONE'
                    logger.info("Gripper action completed.")

            # --- 状态 4: 结束 ---
            elif self.state == 'DONE':
                self.state = 'MOVING'
                break
            
            fps_spend_time = time.perf_counter() - fps_start_time
            time.sleep(1 / 10 - fps_spend_time)

            elapsed_total = time.time() - start_time
            if elapsed_total > self.timeout:
                logger.error(f"Motion Timeout! Elapsed: {elapsed_total:.2f}s")
                return False
        
        return True
    
    def update(self):
        start_loop_t = time.perf_counter()

        # if hasattr(self.robot, "teleop_step"):
        #     observation, action = self.robot.teleop_step(record_data=True)

        #     self.set_observation(observation)
        #     self.set_obs_action(action)

        # else:
        observation = self.robot.get_observation()
        self.set_observation(observation)

        # status = safe_update_status(self.robot)
        # self.set_status(status)

        # pre_action = self.get_pre_action()
        # if pre_action is not None:
        #     action = self.robot.send_action(pre_action)
            # action = {"action": action}

        dt_s = time.perf_counter() - start_loop_t
        if self.fps is not None:
            time.sleep(1 / self.fps - dt_s)

        # log_control_info(self.robot, dt_s, fps=self.fps)

    def set_pre_action(self, value: Union[Any, Dict[str, Any]]):
        with self.data_lock:
            if value is None:
                return
            self.pre_action = value.copy()

    def set_obs_action(self, value: Union[Any, Dict[str, Any]]):
        with self.data_lock:
            if value is None:
                return
            self.obs_action = value.copy()

    def set_observation(self, value: Union[Any, Dict[str, Any]]):
        with self.data_lock:
            if value is None:
                return
            self.observation = value.copy()

    def set_status(self, value: Optional[str]):
        with self.data_lock:
            if value is None:
                return
            self.status = value

    def get_pre_action(self) -> Union[Any, Dict[str, Any]]:
        with self.data_lock:
            if self.pre_action is None:
                return None
            return self.pre_action.copy()

    def get_obs_action(self) -> Union[Any, Dict[str, Any]]:
        with self.data_lock:
            if self.obs_action is None:
                return None
            return self.obs_action.copy()

    def get_observation(self) -> Union[Any, Dict[str, Any]]:
        with self.data_lock:
            if self.observation is None:
                return None
            return self.observation.copy()

    def get_status(self) -> Optional[str]:
        with self.data_lock:
            if self.status is None:
                return None
            return self.status

        