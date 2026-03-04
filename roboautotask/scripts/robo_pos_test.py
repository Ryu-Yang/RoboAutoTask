import sys
import cv2
import argparse
import logging_mp
import time
import numpy as np
import asyncio
import threading


from dataclasses import asdict, dataclass
from pprint import pformat

from lerobot.robots import RobotConfig
from robodriver.utils import parser
from robodriver.core.ros2thread import ROS2_NodeManager
from robodriver.utils.import_utils import register_third_party_devices
from robodriver.robots.utils import make_robot_from_config, Robot
from robodriver.core.coordinator import Coordinator

from roboautotask.robot.daemon import Daemon
from roboautotask.core.operator import Operator
from roboautotask.core.operator import OperatorConfig
from roboautotask.core.motion import MotionExecutor
from roboautotask.core.motion import MotionConfig

from roboautotask.utils.pose import save_pose_to_file

from roboautotask.estimation.target import TargetDetection
from roboautotask.camera.realsense import RealsenseCameraClientNode
# from roboautotask.robot.driver import InterpolationDriverNode
from roboautotask.configs.robot import ROBOT_START_POS, ROBOT_START_ORI
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
    operator: OperatorConfig
    motion: MotionConfig


async def record(daemon, stop_event):
    """后台异步任务：从队列消费图像并发送"""

    coordinator = Coordinator(daemon, None)
    await coordinator.start()
    coordinator.stream_info(daemon.cameras_info)
    await coordinator.update_stream_info_to_server()

    try:
        while not stop_event.is_set():
            daemon.update()
            observation = daemon.get_observation()
            tasks = []
            if observation is not None:
                for key in observation:
                    if "image" in key and "depth" not in key:
                        img = cv2.cvtColor(observation[key], cv2.COLOR_RGB2BGR)
                        tasks.append(coordinator.update_stream_async(key, img))

            if tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*tasks, return_exceptions=True), timeout=0.2
                    )
                except asyncio.TimeoutError:
                    pass

            
            else:
                logger.warning("observation is none")
            
            # cv2.waitKey(1)
            await asyncio.sleep(0)
    finally:
        await coordinator.stop()


@parser.wrap()
def run(cfg: ControlPipelineConfig):
    logger.info(pformat(asdict(cfg)))

    ros2_node_manager = ROS2_NodeManager()

    try:
        robot: Robot = make_robot_from_config(cfg.robot)
    except Exception as e:
        logger.critical(f"Failed to create robot: {e}")
        raise

    logger.info("Make robot success")
    logger.info(f"robot.type: {robot.robot_type}")

    if hasattr(robot, "get_node"):
        robot_node = robot.get_node()
    else:
        logger.error("Can't get ros2 node from robot")
    # driver_node = InterpolationDriverNode(0)

    ros2_node_manager.add_node(robot_node)
    ros2_node_manager.start()

    daemon = Daemon(robot)
    daemon.start()
    target_detection = TargetDetection()

    print("executing")
    daemon.execute_motion(ROBOT_START_POS, ROBOT_START_ORI, 30, 100)
    print("executed")


def main():
    register_third_party_devices()
    logger.info(f"Registered robot types: {list(RobotConfig._choice_registry.keys())}")
    run()


if __name__ == "__main__":
    main()


'''uasage
python roboautotask/scripts/robo_pos_test.py \
    --robot.type=galaxea_lite_eepose_ros2 \
    --operator.task_id=1188 \
    --operator.user=xuyihao \
    --operator.password=Xuyihao@2026 \
    --motion.config_path=motions.yaml
'''
