# RoboAutoTask
This repository is used for the automated task collection of RoboXStudio and RoboDriver.

# Quick use

```
uv venv -p 3.10
source .venv/bin/activate
```

```
uv pip install -e .
playwright install
```

install ros2 realsense
```
sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
```

start ros2 realsense
```bash
# 先看sn
realsense-viewer
# 启动后如果报错udev。请执行打开的窗口提醒中的命令，最后个命令需要手动加sudo。
```

```bash
ros2 launch realsense2_camera rs_launch.py \
    serial_no:="'<your camera sn>'" \
    camera_name:='camera_head' \
    align_depth.enable:=true \
    depth_module.depth_profile:=640,480,30 \
    rgb_camera.color_profile:=640,480,30
```
```bash
roboautotask-run --robot.type=galaxea_lite_eepose_ros2 --operator.task_id=1188 --operator.user=xuyihao --operator.password=Xuyihao@2026 --motion.config_path=motions.yaml
```