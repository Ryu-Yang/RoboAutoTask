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

galaxea
```bash
roboautotask-run --robot.type=galaxea_lite_eepose_ros2 --operator.task_id=1188  --operator.reset_task_id=1188 --operator.user=xuyihao --operator.password=Xuyihao@2026 --motion.config_path=motions.yaml --motion.arm=left --motion.item_id_record=6 --motion.item_id_reset=-5
```

agilex
```bash
roboautotask-run --robot.type=agilex_aloha_eepose_mix --operator.task_id=1212 --operator.reset_task_id=1215 --operator.user=shenyanci --operator.password=SHENyanci@930 --motion.config_path=motions_aloha_opencv.yaml  --motion.arm=left --motion.item_id_record=2 --motion.item_id_reset=-3
```

如果使用过程中遇到了卡死或VSCode闪退的情况，请清空 `/dev/shm/*`:

```
sudo rm -rf /dev/shm/*
```

如果出现了储存空间占满的情况，可以手动运行 数据清理 脚本：

运行的话，新开个终端，激活环境：

```
source .venv/bin/activate
```

删除3天以前的
```bash
python roboautotask/scripts/clean_data.py --root /home/<这里换成对应的用户名，括号删除>/DoRobot/ --days 3
```