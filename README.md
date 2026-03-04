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
    serial_no:="'341522300888'" \
    camera_name:='camera_head' \
    align_depth.enable:=true \
    depth_module.depth_profile:=640,480,30 \
    rgb_camera.color_profile:=640,480,30
```
```bash
roboautotask-run --robot.type=galaxea_lite_eepose_ros2 --operator.task_id=1188 --operator.user=xuyihao --operator.password=Xuyihao@2026 --motion.config_path=motions.yaml
```
```
### Error querying database. Cause: java.sql.SQLException: connection disabled ### The error may exist in URL [jar:file:/app/app.jar!/BOOT-INF/lib/eai-system-1.0.0-SNAPSHOT.jar!/mapper/system/SysDictDataMapper.xml] ### The error may involve org.baai.data.eai.system.mapper.SysDictDataMapper.selectDictDataByTypes ### The error occurred while executing a query ### SQL: select dict_code, dict_sort, dict_label, dict_value, dict_type, css_class, list_class, is_default, status, create_by, create_time, remark from sys_dict_data where status = '0' and dict_type in ( ? , ? , ? , ? ) order by dict_type, dict_sort asc ### Cause: java.sql.SQLException: connection disabled ; uncategorized SQLException; SQL state [null]; error code [0]; connection disabled
```