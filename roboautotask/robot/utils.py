import numpy as np
from typing import Any

from roboautotask.configs.svd import CALIB_R, CALIB_T
from roboautotask.configs.robot import ROBOT_START_POS

from roboautotask.utils import math


def transform_cam_to_robot(point_cam):
    """
    对应原 SVD_get_target.py
    输入: 相机坐标系点 [x, y, z]
    输出: 机器人基座坐标系点 [x, y, z]
    """
    p_cam = np.array(point_cam)
    if p_cam.shape != (3,):
        p_cam = p_cam.reshape(3)
        
    # Robot = R @ Cam + t
    point_robot = (CALIB_R @ p_cam) + CALIB_T
    return point_robot

def get_target_flange_pose(target_obj_pos, offset_x, tilt_angle_deg=45.0):
    """
    计算末端法兰的最终位置和姿态。
    
    坐标系说明：
    1. X轴：由 Home 指向 Target 的水平方向，再向下俯仰 tilt_angle_deg 度。
    2. Y轴：叉乘得到，符合右手定则（保持水平，不随俯仰变化）。
    3. Z轴：由 X cross Y 重新计算，保证三轴正交。
    
    Args:
        target_obj_pos: 目标物体在机器人基座坐标系下的位置 [x, y, z]
        offset_x: 沿接近方向的回退距离（末端不直接到达目标，提前 offset_x 停下）
        tilt_angle_deg: 接近方向相对水平面向下倾斜的角度（默认 45°）
                        0° = 水平抓取，90° = 竖直向下抓取
    """
    # 1. 目标点与参考点
    P_target = np.array(target_obj_pos, dtype=np.float64)
    P_home = np.array(ROBOT_START_POS, dtype=np.float64)
    
    # 2. 计算水平接近方向（投影到 XY 平面）
    vec_horiz = P_target - P_home
    vec_horiz[2] = 0.0  # 忽略 Z 分量，只取水平方向
    horiz_dist = np.linalg.norm(vec_horiz)
    
    if horiz_dist < 1e-3:
        # 特殊情况：目标正上方/正下方，默认取 X 轴正方向
        ux_horiz = np.array([1.0, 0.0, 0.0])
    else:
        ux_horiz = vec_horiz / horiz_dist  # 单位化水平方向

    # 3. 将水平方向绕 Y 轴（侧轴）向下俯仰 tilt_angle_deg 度
    #    ux = cos(tilt) * ux_horiz - sin(tilt) * [0,0,1]  (向下为负 Z)
    tilt_rad = np.deg2rad(tilt_angle_deg)
    down_world = np.array([0.0, 0.0, -1.0])  # 向下方向
    ux = np.cos(tilt_rad) * ux_horiz + np.sin(tilt_rad) * down_world
    ux = ux / np.linalg.norm(ux)  # 单位化（理论上已是单位向量，此处保险）

    # 4. 计算 Y 轴（保持水平，不受俯仰影响）
    #    uy = world_up cross ux_horiz（基于水平方向，与俯仰无关）
    up_world = np.array([0.0, 0.0, 1.0])
    uy = np.cross(up_world, ux_horiz)
    norm_y = np.linalg.norm(uy)
    
    if norm_y < 1e-3:
        uy = np.array([0.0, 1.0, 0.0])  # 特殊退化情况
    else:
        uy = uy / norm_y

    # 5. 重新计算正交的 Z 轴
    uz = np.cross(ux, uy)
    uz = uz / np.linalg.norm(uz)
    
    # 6. 构建旋转矩阵 [ux | uy | uz]
    rot_mat = np.eye(3)
    rot_mat[:, 0] = ux  # 前（斜向下）
    rot_mat[:, 1] = uy  # 左
    rot_mat[:, 2] = uz  # 上（随俯仰倾斜）
    
    # 转为四元数 (Scipy format: x, y, z, w)
    final_quat = math.matrix_to_quaternion(rot_mat)
    
    # 7. 计算最终坐标：沿斜向接近方向回退 offset_x
    final_pos = P_target - (ux * offset_x)
    
    return final_pos, final_quat

def get_pose_from_observation(observation: dict[str, Any], filt_name):
    """
    从 RoboDriver 机器人的observation中读取pose
    """
    pos = np.zeros(3)
    quat = np.zeros(4)

    for name, data in observation.items():
        if "pos" in name and "pos_x" in name and filt_name in name:
            pos[0] = float(data)
        elif "pos" in name and "pos_y" in name and filt_name in name:
            pos[1] = float(data)
        elif "pos" in name and "pos_z" in name and filt_name in name:
            pos[2] = float(data)
        if "quat" in name and "quat_x" in name and filt_name in name:
            quat[0] = float(data)
        elif "quat" in name and "quat_y" in name and filt_name in name:
            quat[1] = float(data)
        elif "quat" in name and "quat_z" in name and filt_name in name:
            quat[2] = float(data)
        elif "quat" in name and "quat_w" in name and filt_name in name:
            quat[3] = float(data)

    return pos, quat

def get_pose_from_observation(observation: dict[str, Any], filt_name):
    """
    从 RoboDriver 机器人的observation中读取pose
    """
    pos = np.zeros(3)
    quat = np.zeros(4)

    for name, data in observation.items():
        if "pos" in name and "pos_x" in name and filt_name in name:
            pos[0] = float(data)
        elif "pos" in name and "pos_y" in name and filt_name in name:
            pos[1] = float(data)
        elif "pos" in name and "pos_z" in name and filt_name in name:
            pos[2] = float(data)
        if "quat" in name and "quat_x" in name and filt_name in name:
            quat[0] = float(data)
        elif "quat" in name and "quat_y" in name and filt_name in name:
            quat[1] = float(data)
        elif "quat" in name and "quat_z" in name and filt_name in name:
            quat[2] = float(data)
        elif "quat" in name and "quat_w" in name and filt_name in name:
            quat[3] = float(data)

    return pos, quat

def get_gripper_from_observation(observation: dict[str, Any], filt_name):
    """
    从 RoboDriver 机器人的observation中读取gripper
    """
    gripper = np.zeros(1)

    for name, data in observation.items():
        if "gripper" in name and filt_name in name:
            gripper = float(data)
            break

    return gripper

def create_action(pos, quat, gripper, use_arm):
    action = {}

    action[f"leader_{use_arm}_arm_pos_x_m.pos"] = pos[0]
    action[f"leader_{use_arm}_arm_pos_y_m.pos"] = pos[1]
    action[f"leader_{use_arm}_arm_pos_z_m.pos"] = pos[2]
    action[f"leader_{use_arm}_arm_quat_w.pos"] = quat[3]
    action[f"leader_{use_arm}_arm_quat_x.pos"] = quat[0]
    action[f"leader_{use_arm}_arm_quat_y.pos"] = quat[1]
    action[f"leader_{use_arm}_arm_quat_z.pos"] = quat[2]
    action[f"leader_{use_arm}_gripper_percent.pos"] = gripper

    return action
