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

def get_target_flange_pose(target_obj_pos, offset_x):
    """
    计算末端法兰的最终位置和姿态。
    
    坐标系优化说明：
    1. X轴：由 Home 指向 Target (Approach向量)。
    2. Z轴：强制向上 (0,0,1)，保证夹爪不发生绕轴旋转（解决90度旋转问题）。
    3. Y轴：叉乘得到，符合右手定则。
    """
    # 1. 目标点与参考点
    P_target = np.array(target_obj_pos, dtype=np.float64)
    P_home = np.array(ROBOT_START_POS, dtype=np.float64)
    
    # 2. 计算接近方向 (X轴)
    vec_approach = P_target - P_home
    dist = np.linalg.norm(vec_approach)
    if dist < 1e-3:
        ux = np.array([1.0, 0.0, 0.0])
    else:
        ux = vec_approach / dist # 单位化 X 轴

    # --- 构建符合 [+X前, +Y左, +Z上] 的旋转矩阵 ---
    
    # 强制 Z 轴向上 (World Up)
    up_world = np.array([0.0, 0.0, 1.0])
    
    # 计算 Y 轴 = Z cross X (根据右手定则: Z向上 cross X向前 = Y向左)
    uy = np.cross(up_world, ux)
    norm_y = np.linalg.norm(uy)
    
    if norm_y < 1e-3:
        # 特殊情况：如果接近方向刚好是垂直向上/向下的
        uy = np.array([0.0, 1.0, 0.0]) # 默认向左
    else:
        uy = uy / norm_y
        
    # 重新计算正交的 Z 轴 (确保三轴完全正交)
    uz = np.cross(ux, uy)
    uz = uz / np.linalg.norm(uz)
    
    # 构建旋转矩阵 [ux, uy, uz]
    rot_mat = np.eye(3)
    rot_mat[:, 0] = ux  # 前
    rot_mat[:, 1] = uy  # 左
    rot_mat[:, 2] = uz  # 上
    
    # 转为四元数 (Scipy format: x, y, z, w)
    final_quat = math.matrix_to_quaternion(rot_mat)
    
    # --- 计算最终坐标 ---
    # 沿着指向物体的 ux 方向回退 offset_x
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
