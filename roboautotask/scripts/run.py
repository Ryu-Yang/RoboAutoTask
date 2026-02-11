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
    # driver_node = InterpolationDriverNode(0)

    ros2_node_manager.add_node(camera_node)
    ros2_node_manager.add_node(robot_node)
    ros2_node_manager.start()

    # robot.connect()

    save_pose_to_file("./latest_pose.txt", ROBOT_START_POS, ROBOT_START_ORI)
    
    daemon = Daemon(robot)
    daemon.start()
    target_detection = TargetDetection()

    # ===== 新增：创建线程安全队列和停止事件 =====
    # observation_queue = queue.Queue(maxsize=1)  # 只保留最新帧
    stop_event = threading.Event()

    # ===== 启动后台 record 线程 =====
    def run_record():
        asyncio.run(record(daemon, stop_event))

    record_thread = threading.Thread(target=run_record, daemon=True, name="RecordThread")
    record_thread.start()
    logger.info("Started background record thread")

    operator = Operator(cfg.operator)
    motion_executor = MotionExecutor(cfg.motion, daemon, camera_node, target_detection)

    operator.login()



    try:
        while True:
            ### 执行采集任务
            motion_sequence = [[2, -3]]

            # 这是抓取&放置一套流程；抓取物&放置物并发识别；
            for sid in motion_sequence:
                grab_id,place_id = sid

                operator.find_task()
                operator.exec_task()
                operator.start_task()

                result = motion_executor.execute_by_id(grab_id, place_id)
                if result == 0:
                    logger.info(f"Sequence aborted at ID {sid}")
                    break
                elif result == 2:
                    logger.info('无法采集，需要重置！')
                    operator.destroy_task()
                    operator.quit_task()

                    if not motion_executor.reset(grab_id,place_id):
                        logger.info(f"Sequence aborted at reset")
                    logger.info("场景重置完成")
                    break

                elif result == 3:
                    logger.info('运动时间超时，需要检查目标物位置！')
                    operator.destroy_task()
                    operator.quit_task()
                    # 加按键等待
                    # --- 新增：CV2 按键等待功能 ---
                    # 创建一个黑色的画布
                    img = np.zeros((400, 800, 3), np.uint8)
                    # 添加提示文字
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(img, 'TIMEOUT: Check object position!', (50, 150), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    cv2.putText(img, 'Press [C] to Continue / [ESC] to Quit', (50, 250), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    
                    cv2.namedWindow('Robot_Alert', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('Robot_Alert', img)
                    
                    logger.info("等待用户按键反馈...")
                    
                    while True:
                        # 等待按键输入，1ms检查一次
                        key = cv2.waitKey(1) & 0xFF
                        
                        # 如果按下 'c' 或 'C'
                        if key == ord('c') or key == ord('C'):
                            logger.info("用户按下 C，准备重新开始循环...")
                            cv2.destroyWindow('Robot_Alert')
                            break # 跳出当前的 key 等待循环
                        
                        # 如果按下 ESC
                        elif key == 27:
                            logger.info("用户按下 ESC，终止任务")
                            cv2.destroyWindow('Robot_Alert')
                            raise SystemExit # 直接退出程序

                    # continue 将会跳过本次 while True 的剩余逻辑，进入下一次任务循环
                    continue



                operator.complete_task()
                operator.commit_task()
                operator.quit_task()

                logger.info("任务采集完成")

                ### 执行场景重置
                # motion_sequence = [2, 1, 0]

                if not motion_executor.reset(grab_id,place_id):
                    logger.info(f"Sequence aborted at reset")
                    break
                
                logger.info("场景重置完成")

                time.sleep(3)

    finally:
        operator.stop()
        motion_executor.go_home()
        if ros2_node_manager is not None:
            ros2_node_manager.stop()


def main():
    register_third_party_devices()
    logger.info(f"Registered robot types: {list(RobotConfig._choice_registry.keys())}")
    run()


if __name__ == "__main__":
    main()
