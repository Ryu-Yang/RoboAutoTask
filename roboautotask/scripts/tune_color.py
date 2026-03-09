"""
可视化 HSV 阈值调节工具

用法示例：
  roboautotask-tune-color --item "an orange object"
  roboautotask-tune-color --item cup --config motions.yaml

操作说明（界面打开后）：
  拖动 Trackbar 实时调整 H/S/V 上下限
  s  ── 将当前阈值保存到 motions.yaml 并退出
  q  ── 不保存，直接退出

配置将写入对应 item 的 hsv_config 字段：
  items:
    3:
      name: "cup"
      hsv_config:
        h_min: 10
        h_max: 30
        ...
"""

import argparse
import logging_mp
import yaml
import os

from robodriver.core.ros2thread import ROS2_NodeManager
from roboautotask.camera.realsense import RealsenseCameraClientNode
from roboautotask.estimation.opencv_detector import ColorBlobDetector
from roboautotask.configs.topic import (
    CAMERA_COLOR_SUB_TOPIC,
    CAMERA_DEPTH_SUB_TOPIC,
    CAMERA_COLOR_INFO_SUB_TOPIC,
    CAMERA_DEPTH_INFO_SUB_TOPIC,
)

logging_mp.basic_config(level=logging_mp.INFO)
logger = logging_mp.get_logger(__name__)


def _list_items(config_path: str) -> None:
    """打印 motions.yaml 中所有可用的 item 名称。"""
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        items = data.get("items", {})
        print("\nAvailable items in config:")
        for k, v in items.items():
            if not isinstance(v, dict):
                continue
            name = v.get("name", "")
            label = v.get("label", "")
            has_hsv = "hsv_config" in v
            status = "[HSV configured]" if has_hsv else "[no HSV config]"
            print(f"  ID {k:>4}: name={name!r}  label={label!r}  {status}")
        print()
    except Exception as e:
        logger.error(f"Failed to list items: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Visually tune HSV color thresholds for OpenCV-based object detection."
    )
    parser.add_argument(
        "--item",
        type=str,
        default=None,
        help="Item name or label to tune (as defined in motions.yaml).",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="motions.yaml",
        help="Path to motions.yaml (default: motions.yaml).",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all available items in the config file and exit.",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Do not save the tuned config back to the yaml file.",
    )
    args = parser.parse_args()

    config_path = os.path.abspath(args.config)

    if args.list:
        _list_items(config_path)
        return

    if args.item is None:
        parser.print_help()
        print("\nError: --item is required. Use --list to see available items.\n")
        return

    logger.info(f"Starting HSV tuner for item: '{args.item}'")
    logger.info(f"Config file: {config_path}")

    ros2_node_manager = ROS2_NodeManager()

    camera_node = RealsenseCameraClientNode(
        color_topic=CAMERA_COLOR_SUB_TOPIC,
        depth_topic=CAMERA_DEPTH_SUB_TOPIC,
        color_info_topic=CAMERA_COLOR_INFO_SUB_TOPIC,
        depth_info_topic=CAMERA_DEPTH_INFO_SUB_TOPIC,
    )
    ros2_node_manager.add_node(camera_node)
    ros2_node_manager.start()

    detector = ColorBlobDetector(config_path=config_path)

    try:
        proceeded = detector.tune_for_task(
            [args.item],
            camera=camera_node,
            save=not args.no_save,
        )
        if proceeded:
            logger.info(f"Final config for '{args.item}': {detector.get_hsv_config(args.item)}")
        else:
            logger.info("Tuning exited without saving.")
    finally:
        ros2_node_manager.stop()


if __name__ == "__main__":
    main()
