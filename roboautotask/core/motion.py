import cv2
import time
import numpy as np
import yaml
import logging_mp
from dataclasses import dataclass

from roboautotask.estimation.sensor import capture_target_coordinate
# from roboautotask.robot.driver import execute_motion
from roboautotask.robot.daemon import Daemon
from roboautotask.robot.utils import transform_cam_to_robot, get_target_flange_pose

from roboautotask.configs.robot import ROBOT_START_POS, ROBOT_START_ORI

from roboautotask.utils.pose import load_pose_from_file
from roboautotask.utils.math import generate_random_points_around_center, obj_is_in_placement
from roboautotask.scripts.robo_reset import reset
from roboautotask.camera.realsense import RealsenseCameraClientNode
from roboautotask.estimation.target import TargetDetection
from scipy.spatial.transform import Rotation

logger = logging_mp.get_logger(__name__)


@dataclass()
class MotionConfig:
    config_path: str = "tasks.yaml"


def view_eular(quaternion_xyzw):
    # 将四元数转换为欧拉角 (弧度)
    rotation = Rotation.from_quat(quaternion_xyzw)
    euler_angles = rotation.as_euler('xyz', degrees=True) # 外旋xyz顺序，对应内旋zyx

    return euler_angles


class MotionExecutor:
    def __init__(self, cfg: MotionConfig, daemon: Daemon, camera: RealsenseCameraClientNode, target_detection: TargetDetection):
        self.daemon = daemon
        self.target_detection = target_detection
        self.camera = camera
        self.start_center = [0.0, 0.0, 0.0]
        with open(cfg.config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

    # def _get_current(self):
    #     return load_pose_from_file("latest_pose.txt")

    def execute_by_id(self, grab_id,place_id):
        grab_item = self.cfg['items'].get(grab_id)
        place_item = self.cfg['items'].get(place_id)
        if (not grab_item) or (not place_item): return 0

        logger.info(f">>> Task: grab_item['name'] : {grab_item['name']} (ID: {grab_id})")
        logger.info(f">>> Task: grab_item['label'] : {grab_item['label']} (ID: {grab_id})")

        # 1. 定位：获取物体在基座坐标系下的原始位置
        if 'label' in grab_item:
            cam_point = self.target_detection.capture_target_coordinate(target_class = grab_item['label'], camera=self.camera)
            if cam_point is None: return 0
            robot_point_raw = transform_cam_to_robot(cam_point)
        else:
            robot_point_raw = np.array(grab_item['pos'], dtype=float)
        if self.start_center == [0.0, 0.0, 0.0]:
            self.start_center = robot_point_raw.tolist()
            logger.info(f'識別到初始位置:{self.start_center}')


        # 获取放置物在基座坐标系下的原始位置
        logger.info(f">>> Task: {place_item['name']} (ID: {place_id})")
        if 'label' in place_item:
            place_cam_point = self.target_detection.capture_target_coordinate(target_class = place_item['label'], camera=self.camera)
            if place_cam_point is None: return 0
            place_robot_point_raw = transform_cam_to_robot(place_cam_point)
        else:
            place_robot_point_raw = np.array(place_item['pos'], dtype=float)
        
        # ------------- 把此时的放置物体的坐标保存到文件里 -------------
        line = place_robot_point_raw.tolist()
        line = ' '.join(map(str, line))
        with open('palced_obj_pos.txt', 'w') as f:
            f.write(line)
        # # ---------------- 判断是否需要采集数据 -------------------
        # with open('palced_obj_size.txt', 'r') as f:
        #     line = f.readline().strip()  # 读第一行并去除首尾空白（包括换行符）
        #     real_w, real_h = map(float, line.split())
        # 如果物体在放置物里面的话需要丢弃重采
        if obj_is_in_placement(robot_point_raw, place_robot_point_raw):
            return 2


        ### 运动到物体位置
        z_offset = grab_item.get('offsets', {}).get('z', 0)
        robot_point_raw[2] += z_offset
        off_x = grab_item.get('offsets', {}).get('x', 0)
        
        logger.info(f"target_pos obj: {robot_point_raw}")
        final_pos, final_quat = get_target_flange_pose(
            robot_point_raw, 
            offset_x=off_x
        )
        final_eular = view_eular(final_quat)
        logger.info(f"Moving to target. Base_Z_Offset: {z_offset}, Tool_X_Offset: {off_x}, final_pos: {final_pos}, final_quat: {final_quat}, final_eular: {final_eular}")
        
        if not self.daemon.execute_motion(final_pos, final_quat, 1200, grab_item['gripper_pos']):
            reset(self.daemon)
            return 3
        

        ### 运动到放置位置
        place_z_offset = place_item.get('offsets', {}).get('z', 0)
        place_robot_point_raw[2] += place_z_offset
        place_off_x = place_item.get('offsets', {}).get('x', 0)

        logger.info(f"target_pos place: {place_robot_point_raw}")
        place_final_pos, place_final_quat = get_target_flange_pose(
            place_robot_point_raw,
            offset_x=place_off_x
        )
        place_final_eular = view_eular(place_final_quat)
        logger.info(f"Moving to target. Base_Z_Offset: {place_z_offset}, Tool_X_Offset: {place_off_x}, place_final_pos: {place_final_pos}, place_final_quat:{place_final_quat}, place_final_eular: {place_final_eular}")
        
        self.daemon.execute_motion(place_final_pos, place_final_quat, 1200, place_item['gripper_pos'])

        return self.go_home()

    def reset(self, grab_id, place_id):

        grab_item = self.cfg['items'].get(grab_id)
        place_item = self.cfg['items'].get(place_id)
        if (not grab_item)or(not place_item): return 0

        print(f">>> Task: {grab_item['name']} (ID: {grab_id})")

        # 1. 定位：获取抓取物体在基座坐标系下的原始位置
        if 'label' in grab_item:
            cam_point = self.target_detection.capture_target_coordinate(target_class=grab_item['label'], camera=self.camera)
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

        # 在重置里面取消上面对放置物的识别，基座标点直接照搬放置时识别的位置
        with open('palced_obj_pos.txt', 'r') as f:
            line = f.readline().strip()  # 读第一行并去除首尾空白（包括换行符）
            place_x,place_y,place_z = map(float, line.split())
        place_robot_point_raw = [place_x,place_y,place_z]
        place_robot_point_raw = np.array(place_robot_point_raw)

        # # ---------------- 判断是否需要重置 -------------------
        # with open('palced_obj_size.txt', 'r') as f:
        #     line = f.readline().strip()  # 读第一行并去除首尾空白（包括换行符）
        #     real_w, real_h = map(float, line.split())
        # print(robot_point_raw, place_robot_point_raw,real_w,real_h)
        # 如果物体在放置物里面的话需要丢弃重采
        if not obj_is_in_placement(robot_point_raw, place_robot_point_raw):
            return 2
        

        # --------------- 判断通过了以后再进行随机点的生成 ------------------
        # 优先使用用户在相机画面中手动框选的区域
        region_point = self._sample_reset_point_from_region()
        if region_point is not None:
            place_robot_point_raw = region_point
        else:
            # 备用：使用原有随机中心点生成方法
            place_robot_point_raw = generate_random_points_around_center(
                center_point=self.start_center,
                target_point=place_robot_point_raw.tolist(),
                rectangle_width=0.10,
                rectangle_length=0.10,
                rectangle_height=0.01,
                exclusion_radius=0.1,
                num_points=1,
                distribution_power=0.5)[0]

        # # 2. 获取当前起始位姿
        # start_pos, start_quat = self._get_current()

        # 3. Z轴偏移处理（基座坐标系下直接叠加）
        # 比如放置在盘子上方，直接修改 robot_point_raw 的 Z 值
        z_offset = grab_item.get('offsets', {}).get('z', 0)
        robot_point_raw[2] += z_offset

        place_z_offset = place_item.get('offsets', {}).get('z', 0)
        place_robot_point_raw[2] += place_z_offset + 0.02

        # 4. 计算末端法兰位姿
        # offset_x 依然用于处理夹爪/物体的距离补偿
        off_x = grab_item.get('offsets', {}).get('x', 0)
        
        final_pos, final_quat = get_target_flange_pose(
            robot_point_raw, 
            offset_x=off_x
        )

        place_off_x = place_item.get('offsets', {}).get('x', 0)
        
        place_final_pos, place_final_quat = get_target_flange_pose(
            place_robot_point_raw, 
            offset_x=place_off_x
        )

        # 5. 执行运动与夹爪
        logger.info(f"Moving to target. Base_Z_Offset: {z_offset}, Tool_X_Offset: {off_x}, final_quat:{final_quat}")
        self.daemon.execute_motion(final_pos, final_quat, 1200, grab_item['gripper_pos'])
        # robot_driver.set_gripper_position(item['gripper_pos'])

        logger.info(f"Moving to target. Base_Z_Offset: {place_z_offset}, Tool_X_Offset: {place_off_x}, final_quat:{final_quat}")
        self.daemon.execute_motion(place_final_pos, place_final_quat, 1200, place_item['gripper_pos'])
        

        return self.go_home()

    def setup_reset_region(self):
        """打开相机画面，让用户手动框选重置放置区域，保存到 reset_region.txt
        
        操作说明:
          - 鼠标左键拖动画出矩形区域
          - 按 Enter / 空格键 确认选择
          - 按 C 取消
        """
        logger.info("将弹出相机画面，请用鼠标拖动框选重置放置区域，按 Enter 确认，按 C 取消")

        # 等待相机就绪
        color_img = None
        for _ in range(50):
            color_img = self.camera.color_image
            if color_img is not None:
                break
            time.sleep(0.1)

        if color_img is None:
            logger.error("无法获取相机画面，放弃区域选择")
            return False

        # 转 BGR 供 cv2 显示
        display_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)

        cv2.namedWindow("Select Reset Region", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Select Reset Region", 800, 600)
        roi = cv2.selectROI("Select Reset Region", display_img, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select Reset Region")

        x, y, w, h = roi
        if w == 0 or h == 0:
            logger.warning("未选择有效区域，将使用原有随机方法")
            return False

        u1, v1, u2, v2 = x, y, x + w, y + h
        with open('reset_region.txt', 'w') as f:
            f.write(f"{u1} {v1} {u2} {v2}\n")

        # 在图上画出选定区域并预览
        preview = display_img.copy()
        cv2.rectangle(preview, (u1, v1), (u2, v2), (0, 255, 0), 2)
        cv2.putText(preview, f"Reset region: ({u1},{v1})-({u2},{v2})",
                    (u1, max(v1 - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.namedWindow("Reset Region Preview", cv2.WINDOW_NORMAL)
        cv2.imshow("Reset Region Preview", preview)
        logger.info(f"重置区域已保存: u1={u1}, v1={v1}, u2={u2}, v2={v2}")
        logger.info("按任意键关闭预览窗口")
        cv2.waitKey(0)
        cv2.destroyWindow("Reset Region Preview")
        return True

    def _sample_reset_point_from_region(self, max_attempts: int = 50):
        """从 reset_region.txt 指定的像素区域内随机采样一个有效的机器人坐标。

        返回
        ------
        np.ndarray [x, y, z] 机器人基座标系坐标，或 None（文件不存在/深度无效）
        """
        try:
            with open('reset_region.txt', 'r') as f:
                u1, v1, u2, v2 = map(int, f.readline().strip().split())
        except FileNotFoundError:
            logger.warning("reset_region.txt 不存在，将使用原有随机方法")
            return None

        for attempt in range(max_attempts):
            u = int(np.random.uniform(u1, u2))
            v = int(np.random.uniform(v1, v2))

            # pixel_to_3d 读取内部深度图，返回相机坐标系 [x_right, y_down, z_front]
            point_3d = self.camera.pixel_to_3d(u, v)
            if point_3d is None:
                continue

            # 与 get_3d_pts 相同的坐标系转换：+X=前, +Y=左, +Z=上
            x_cam, y_cam, z_cam = point_3d
            cam_point = np.array([z_cam, -x_cam, -y_cam])

            robot_point = transform_cam_to_robot(cam_point)
            logger.info(f"区域采样点 pixel=({u},{v}) -> robot={robot_point.tolist()}")
            return robot_point

        logger.error(f"经过 {max_attempts} 次尝试后未能在区域内找到有效深度点")
        return None

    def go_home(self):
        self.daemon.execute_motion(ROBOT_START_POS, ROBOT_START_ORI, 1200, 100)
        # robot_driver.set_gripper_position(100)
        return 1
    
    def go_random_pose(self, center_item_id = -3):
        rand_pos = []

        item = self.cfg['items'].get(center_item_id)
        if not item: return False

        if 'label' in item:
            cam_point = self.target_detection.capture_target_coordinate(target_class=item['label'], camera=self.camera)
            if cam_point is None: return False
            robot_point_raw = transform_cam_to_robot(cam_point)

            rand_pos = generate_random_points_around_center(center_point=robot_point_raw.tolist())[0]
            # 从yaml获取盘子的zoffset，避免放置平面高度过低
            z_offset = item.get('offsets', {}).get('z', 0)
            rand_pos[2] += z_offset

        else:
            return False

        logger.info(f"rand_pos: {rand_pos} ")
        final_pos, final_quat = get_target_flange_pose(rand_pos, offset_x=0.08)

        logger.info(f"final_pos: {final_pos} , final_quat: {final_quat}")
        self.daemon.execute_motion(final_pos, final_quat, 1200, 100)
        return True
