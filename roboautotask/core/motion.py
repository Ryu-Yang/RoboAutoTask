import numpy as np
import yaml
import logging_mp

from roboautotask.estimation.sensor import capture_target_coordinate
from roboautotask.robot.driver import execute_motion
from roboautotask.robot.utils import transform_cam_to_robot, get_target_flange_pose

from roboautotask.configs.robot import ROBOT_START_POS, ROBOT_START_ORI

from roboautotask.utils.pose import load_pose_from_file
from roboautotask.utils.math import generate_random_points_around_center


logger = logging_mp.get_logger(__name__)


class MotionExecutor:
    def __init__(self, config_path="tasks.yaml"):
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

    def _get_current(self):
        return load_pose_from_file("latest_pose.txt")

    def execute_by_id(self, action_id):
        if action_id == 0: 
            return self.go_home()

        if action_id == 1: 
            return self.go_random_pose()

        item = self.cfg['items'].get(action_id)
        if not item: return False

        print(f">>> Task: {item['name']} (ID: {action_id})")

        # 1. 定位：获取物体在基座坐标系下的原始位置
        if 'label' in item:
            cam_point = capture_target_coordinate(item['label'])
            if cam_point is None: return False
            robot_point_raw = transform_cam_to_robot(cam_point)
        else:
            robot_point_raw = np.array(item['pos'], dtype=float)

        # 2. 获取当前起始位姿
        start_pos, start_quat = self._get_current()

        # 3. Z轴偏移处理（基座坐标系下直接叠加）
        # 比如放置在盘子上方，直接修改 robot_point_raw 的 Z 值
        z_offset = item.get('offsets', {}).get('z', 0)
        robot_point_raw[2] += z_offset

        # 4. 计算末端法兰位姿
        # offset_x 依然用于处理夹爪/物体的距离补偿
        off_x = item.get('offsets', {}).get('x', 0)
        
        final_pos, final_quat = get_target_flange_pose(
            start_pos, 
            robot_point_raw, 
            offset_x=off_x
        )

        # 5. 执行运动与夹爪
        print(f"Moving to target. Base_Z_Offset: {z_offset}, Tool_X_Offset: {off_x}")
        execute_motion(start_pos, start_quat, final_pos, final_quat, item['gripper_pos'])
        # robot_driver.set_gripper_position(item['gripper_pos'])
        
        return True

    def go_home(self):
        s_p, s_q = self._get_current()
        execute_motion(s_p, s_q, ROBOT_START_POS, ROBOT_START_ORI, 100)
        # robot_driver.set_gripper_position(100)
        return True
    
    def go_random_pose(self, center_item_id = -3):
        rand_pos = []
        s_p, s_q = self._get_current()

        item = self.cfg['items'].get(center_item_id)
        if not item: return False

        if 'label' in item:
            cam_point = capture_target_coordinate(item['label'])
            if cam_point is None: return False
            robot_point_raw = transform_cam_to_robot(cam_point)

            rand_pos = generate_random_points_around_center(center_point=robot_point_raw.tolist())[0]

        else:
            return False

        logger.info(f"rand_pos: {rand_pos} ")
        final_pos, final_quat = get_target_flange_pose(s_p, rand_pos, offset_x=0.08)

        logger.info(f"final_pos: {final_pos} , final_quat: {final_quat}")
        execute_motion(s_p, s_q, final_pos, final_quat, 100)
        return True
    