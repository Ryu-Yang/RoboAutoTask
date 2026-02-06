import numpy as np
import yaml
import logging_mp

from roboautotask.estimation.sensor import capture_target_coordinate
from roboautotask.robot.driver import execute_motion
from roboautotask.robot.utils import transform_cam_to_robot, get_target_flange_pose

from roboautotask.configs.robot import ROBOT_START_POS, ROBOT_START_ORI

from roboautotask.utils.pose import load_pose_from_file
from roboautotask.utils.math import generate_random_points_around_center, obj_is_in_placement


logger = logging_mp.get_logger(__name__)


class MotionExecutor:
    def __init__(self, config_path="tasks.yaml"):
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

    def _get_current(self):
        return load_pose_from_file("latest_pose.txt")

    def execute_by_id(self, grab_id,place_id):
        grab_item = self.cfg['items'].get(grab_id)
        place_item = self.cfg['items'].get(place_id)
        if (not grab_item) or (not place_item): return 0

        print(f">>> Task: {grab_item['name']} (ID: {grab_id})")

        # 1. 定位：获取物体在基座坐标系下的原始位置
        if 'label' in grab_item:
            cam_point = capture_target_coordinate(grab_item['label'])
            if cam_point is None: return 0
            robot_point_raw = transform_cam_to_robot(cam_point)
        else:
            robot_point_raw = np.array(grab_item['pos'], dtype=float)


        # 获取放置物在基座坐标系下的原始位置
        print(f">>> Task: {place_item['name']} (ID: {place_id})")
        if 'label' in place_item:
            place_cam_point = capture_target_coordinate(place_item['label'])
            if place_cam_point is None: return 0
            place_robot_point_raw = transform_cam_to_robot(place_cam_point)
        else:
            place_robot_point_raw = np.array(place_item['pos'], dtype=float)
        
        # ------------- 把此时的放置物体的坐标保存到文件里 -------------
        line = place_robot_point_raw.tolist()
        line = ' '.join(map(str, line))
        with open('palced_obj_pos.txt', 'w') as f:
            f.write(line)
        # ---------------- 判断是否需要采集数据 -------------------
        with open('palced_obj_size.txt', 'r') as f:
            line = f.readline().strip()  # 读第一行并去除首尾空白（包括换行符）
            real_w, real_h = map(float, line.split())
        # 如果物体在放置物里面的话需要丢弃重采
        if obj_is_in_placement(robot_point_raw, place_robot_point_raw,real_w,real_h):
            return 2


        # 2. 获取当前起始位姿
        start_pos, start_quat = self._get_current()

        # 3. Z轴偏移处理（基座坐标系下直接叠加）
        # 比如放置在盘子上方，直接修改 robot_point_raw 的 Z 值
        z_offset = grab_item.get('offsets', {}).get('z', 0)
        robot_point_raw[2] += z_offset

        place_z_offset = place_item.get('offsets', {}).get('z', 0)
        place_robot_point_raw[2] += place_z_offset

        # 4. 计算末端法兰位姿
        # offset_x 依然用于处理夹爪/物体的距离补偿
        off_x = grab_item.get('offsets', {}).get('x', 0)
        
        final_pos, final_quat = get_target_flange_pose(
            start_pos, 
            robot_point_raw, 
            offset_x=off_x
        )


        place_off_x = place_item.get('offsets', {}).get('x', 0)
        
        place_final_pos, place_final_quat = get_target_flange_pose(
            start_pos, 
            place_robot_point_raw, 
            offset_x=place_off_x
        )
        # 5. 执行运动与夹爪
        print(f"Moving to target. Base_Z_Offset: {z_offset}, Tool_X_Offset: {off_x}")
        execute_motion(start_pos, start_quat, final_pos, final_quat, grab_item['gripper_pos'])
        # robot_driver.set_gripper_position(item['gripper_pos'])

        print(f"Moving to target. Base_Z_Offset: {place_z_offset}, Tool_X_Offset: {place_off_x}")
        execute_motion(final_pos, final_quat, place_final_pos, place_final_quat, place_item['gripper_pos'])
        # robot_driver.set_gripper_position(item['gripper_pos'])
        
        return self.go_home()

    def reset(self, grab_id, place_id):

        grab_item = self.cfg['items'].get(grab_id)
        place_item = self.cfg['items'].get(place_id)
        if (not grab_item)or(not place_item): return 0

        print(f">>> Task: {grab_item['name']} (ID: {grab_id})")

        # 1. 定位：获取抓取物体在基座坐标系下的原始位置
        if 'label' in grab_item:
            cam_point = capture_target_coordinate(grab_item['label'])
            if cam_point is None: return 0
            robot_point_raw = transform_cam_to_robot(cam_point)
        else:
            robot_point_raw = np.array(grab_item['pos'], dtype=float)

        # # 获取放置位物体在基座标系下的原始位置,并产生随机位置
        # print(f">>> Task: {place_item['name']} (ID: {place_id})")
        # if 'label' in place_item:
        #     place_cam_point = capture_target_coordinate(place_item['label'])
        #     if place_cam_point is None: return 0
        #     place_robot_point_raw = transform_cam_to_robot(place_cam_point)
        # else:
        #     place_robot_point_raw = np.array(place_item['pos'], dtype=float)
        # place_robot_point_raw = generate_random_points_around_center(center_point=place_robot_point_raw.tolist())[0]

        # 在重置里面删除对放置物的识别，基座标点直接照搬放置时识别的位置
        with open('palced_obj_pos.txt', 'r') as f:
            line = f.readline().strip()  # 读第一行并去除首尾空白（包括换行符）
            place_x,place_y,place_z = map(float, line.split())
        place_robot_point_raw = [place_x,place_y,place_z]
        place_robot_point_raw = np.array(place_robot_point_raw)

        # ---------------- 判断是否需要重置 -------------------
        with open('palced_obj_size.txt', 'r') as f:
            line = f.readline().strip()  # 读第一行并去除首尾空白（包括换行符）
            real_w, real_h = map(float, line.split())
        print(robot_point_raw, place_robot_point_raw,real_w,real_h)
        # 如果物体在放置物里面的话需要丢弃重采
        if not obj_is_in_placement(robot_point_raw, place_robot_point_raw,real_w,real_h):
            return 2
        

        # --------------- 判断通过了以后再进行随机点的生成 ------------------
        place_robot_point_raw = generate_random_points_around_center(center_point=place_robot_point_raw.tolist())[0]

        # 2. 获取当前起始位姿
        start_pos, start_quat = self._get_current()

        # 3. Z轴偏移处理（基座坐标系下直接叠加）
        # 比如放置在盘子上方，直接修改 robot_point_raw 的 Z 值
        z_offset = grab_item.get('offsets', {}).get('z', 0)
        robot_point_raw[2] += z_offset

        place_z_offset = place_item.get('offsets', {}).get('z', 0)
        place_robot_point_raw[2] += place_z_offset

        # 4. 计算末端法兰位姿
        # offset_x 依然用于处理夹爪/物体的距离补偿
        off_x = grab_item.get('offsets', {}).get('x', 0)
        
        final_pos, final_quat = get_target_flange_pose(
            start_pos, 
            robot_point_raw, 
            offset_x=off_x
        )

        place_off_x = place_item.get('offsets', {}).get('x', 0)
        
        place_final_pos, place_final_quat = get_target_flange_pose(
            start_pos, 
            place_robot_point_raw, 
            offset_x=place_off_x
        )

        # 5. 执行运动与夹爪
        print(f"Moving to target. Base_Z_Offset: {z_offset}, Tool_X_Offset: {off_x}")
        execute_motion(start_pos, start_quat, final_pos, final_quat, grab_item['gripper_pos'])
        # robot_driver.set_gripper_position(item['gripper_pos'])

        print(f"Moving to target. Base_Z_Offset: {place_z_offset}, Tool_X_Offset: {place_off_x}")
        execute_motion(final_pos, final_quat, place_final_pos, place_final_quat, place_item['gripper_pos'])
        

        return self.go_home()

    def go_home(self):
        s_p, s_q = self._get_current()
        execute_motion(s_p, s_q, ROBOT_START_POS, ROBOT_START_ORI, 100)
        # robot_driver.set_gripper_position(100)
        return 1
    
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
            # 从yaml获取盘子的zoffset，避免放置平面高度过低
            z_offset = item.get('offsets', {}).get('z', 0)
            rand_pos[2] += z_offset

        else:
            return False

        logger.info(f"rand_pos: {rand_pos} ")
        final_pos, final_quat = get_target_flange_pose(s_p, rand_pos, offset_x=0.08)

        logger.info(f"final_pos: {final_pos} , final_quat: {final_quat}")
        execute_motion(s_p, s_q, final_pos, final_quat, 100)
        return True
    