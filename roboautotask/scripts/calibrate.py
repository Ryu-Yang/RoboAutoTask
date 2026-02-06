import cv2
import time
import datetime
import logging_mp
import numpy as np

from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat

from lerobot.robots import RobotConfig
from robodriver.core.ros2thread import ROS2_NodeManager

from robodriver.robots.utils import make_robot_from_config, Robot
from robodriver.utils import parser
from robodriver.utils.import_utils import register_third_party_devices

from roboautotask.robot.utils import get_pose_from_observation
from roboautotask.calibration.detector import ArucoDetector
from roboautotask.calibration.calibrator import Calibrator
from roboautotask.camera.realsense import RealsenseCameraClientNode
from roboautotask.configs.topic import (
    CAMERA_COLOR_SUB_TOPIC,
    CAMERA_DEPTH_SUB_TOPIC,
    CAMERA_COLOR_INFO_SUB_TOPIC,
    CAMERA_DEPTH_INFO_SUB_TOPIC
)


logging_mp.basic_config(level=logging_mp.INFO)
logger = logging_mp.get_logger(__name__)


@dataclass
class ControlPipelineConfig:
    robot: RobotConfig

@parser.wrap()
def calibrate(cfg: ControlPipelineConfig):
    logger.info(pformat(asdict(cfg)))

    ros2_node_manager = ROS2_NodeManager()

    try:
        robot: Robot = make_robot_from_config(cfg.robot)
    except Exception as e:
        logger.critical(f"Failed to create robot: {e}")
        raise

    logger.info("Make robot success")
    logger.info(f"robot.type: {robot.robot_type}")


    out_dir = Path('output')
    out_dir.mkdir(exist_ok=True)
    csv_path = out_dir / f'arm_aruco_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.csv'
    print(f'Saving to: {csv_path.absolute()}')

    calibrator = Calibrator(csv_path)
    detector = ArucoDetector()

    
    camera_node = RealsenseCameraClientNode(
        color_topic=CAMERA_COLOR_SUB_TOPIC,
        depth_topic=CAMERA_DEPTH_SUB_TOPIC,
        color_info_topic=CAMERA_COLOR_INFO_SUB_TOPIC,
        depth_info_topic=CAMERA_DEPTH_INFO_SUB_TOPIC
    )
    if hasattr(robot, "get_node"):
        robot_node = robot.get_node()
    else:
        logger.error("Can't get ros2 node from robot")

    ros2_node_manager.add_node(camera_node)
    ros2_node_manager.add_node(robot_node)
    ros2_node_manager.start()

    robot.connect()

    win = 'ArUco Detection Preview'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 800, 600)
    logger.info('\n' + '='*60 + '\n' + 'CONTROL: r = record, q = quit' + '\n' + '='*60 + '\n')

    show_record_frame = 0

    try:
        while True:
            stat_time = time.perf_counter()
            obs = robot.get_observation()
            arm_pos, arm_ori = get_pose_from_observation(obs, "left")
            
            detector.clean()
            detector.detect(camera_node.color_image)
            for id, marker in detector.latest_markers.items():
                points_3d = []
                valid = True
                for point in marker:
                    px, py = int(point[0]), int(point[1])

                    if not (0 <= px < 640 and 0 <= py < 480):
                        valid = False; break
                    
                    depth = camera_node.get_depth_value(px, py)

                    if depth is None:
                        valid = False; break
                    if depth <= 0 or depth > 5.0:
                        valid = False; break
                    
                    pt3 = camera_node.color_pixel_to_3d(px, py)
                    points_3d.append(pt3)

                if not valid or len(points_3d) != 4:
                    continue

                # 坐标系：+X前 +Y左 +Z上
                center_rs = np.mean(points_3d, axis=0)
                x, y, z = center_rs[2], -center_rs[0], -center_rs[1]
                detector.latest_centers[id] = (x, y, z)
                
            detector.update_view_image()
            
            img = detector.get_view_image()

            if show_record_frame > 0:
                cv2.putText(img, 'RECORDED!', (250, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)

            cv2.imshow(win, img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q') or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                logger.info('Quit requested.')
                break
            if key == ord('r'):
                if calibrator.record_once(arm_pos, arm_ori, detector.latest_centers):
                    show_record_frame = 30
                else:
                    show_record_frame = 0

            show_record_frame -= 1

            spend_time = stat_time - time.perf_counter()
            time.sleep(1/30 - spend_time)
        
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyWindow(win)
        ros2_node_manager.stop()
        logger.info(f'\nAll data saved -> {csv_path.absolute()}')


def main():
    register_third_party_devices()
    logger.info(f"Registered robot types: {list(RobotConfig._choice_registry.keys())}")
    calibrate()


if __name__ == '__main__':
    main()