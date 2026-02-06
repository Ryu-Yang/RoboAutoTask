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
```
# 先看sn
realsense-viewer
ros2 launch realsense2_camera rs_launch.py serial_no:="'xxx'" camera_name:='camera_head'  align_depth.enable:=true
```
