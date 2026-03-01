import numpy as np
from typing import Any
import logging_mp

from roboautotask.configs.svd import CALIB_R, CALIB_T
from roboautotask.configs.robot import ROBOT_START_POS

from roboautotask.utils.math import matrix_to_quaternion

logger = logging_mp.get_logger(__name__)


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
    计算机器人末端法兰的预抓取姿态（位置 + 四元数）。
    
    【几何逻辑】
    1. X 轴 (Approach): 从 Home 指向 Target 的水平方向，并向下俯仰 tilt_angle。
    2. Y 轴 (Side):    强制保持水平（与地面平行），由 世界 Z 轴 × 水平 X 轴 得到。
    3. Z 轴 (Normal):   由 X × Y 得到，垂直于末端法兰平面。
    
    【应用场景】
    适用于需要固定倾斜角度抓取，且要求末端侧向不翻转的场景（如吸盘抓取、斜向插拔）。

    Args:
        target_obj_pos (list/tuple): 目标物体在机器人基座坐标系下的位置 [x, y, z]。
        offset_x (float): 沿接近方向的安全回退距离。
                          最终位置 = 目标点 - (接近方向向量 * offset_x)。
        tilt_angle_deg (float): 接近方向相对水平面的下倾角度。
                                0.0  = 水平接近
                                45.0 = 斜向下 45 度
                                90.0 = 垂直向下
    
    Returns:
        tuple: (final_pos, final_quat)
            - final_pos (np.ndarray): 最终目标位置 [x, y, z]。
            - final_quat (np.ndarray): 最终目标姿态四元数 [x, y, z, w] 或 [w, x, y, z] 
                                       (取决于 math.matrix_to_quaternion 的实现)。
    
    Raises:
        ValueError: 当目标点与起始点重合且无法确定方向时。
    """
    # ------------------------------------------------------------------
    # 1. 数据准备与类型转换
    # ------------------------------------------------------------------
    P_target = np.array(target_obj_pos, dtype=np.float64)
    # 注意：ROBOT_START_POS 需确保已定义，建议改为参数传入以增加函数纯度
    P_home = np.array(ROBOT_START_POS, dtype=np.float64)
    
    # ------------------------------------------------------------------
    # 2. 计算水平接近方向 (确定 Yaw)
    #    忽略高度差，只关心机器人在水平面上应该朝哪边转
    # ------------------------------------------------------------------
    vec_horiz = P_target - P_home
    vec_horiz[2] = 0.0  # 强制 Z 分量为 0，投影到 XY 平面
    horiz_dist = np.linalg.norm(vec_horiz)

    logger.info(f"In get_target_flange_pose --- vec_horiz: {vec_horiz}")
    
    # 处理奇异点：如果目标点就在 Home 正上方或正下方，水平距离为 0
    if horiz_dist < 1e-3:
        # 默认指向世界坐标系 X 轴正方向
        ux_horiz = np.array([1.0, 0.0, 0.0])
    else:
        ux_horiz = vec_horiz / horiz_dist  # 单位化水平方向向量

    # ------------------------------------------------------------------
    # 3. 计算末端 X 轴 (确定 Pitch/Tilt)
    #    在水平方向基础上，绕侧轴向下旋转 tilt_angle_deg
    #    公式：V_new = cos(θ)*V_horiz + sin(θ)*V_down
    # ------------------------------------------------------------------
    tilt_rad = np.deg2rad(tilt_angle_deg)
    down_world = np.array([0.0, 0.0, -1.0])  # 世界坐标系向下方向
 
    # 线性组合得到倾斜后的向量
    ux = np.cos(tilt_rad) * ux_horiz + np.sin(tilt_rad) * down_world
    # 数值稳定性处理：理论上模长为 1，但浮点运算后重新归一化
    ux = ux / np.linalg.norm(ux)

    logger.info(f"In get_target_flange_pose --- ux: {ux}")  

    # ------------------------------------------------------------------
    # 4. 计算末端 Y 轴 (强制保持水平，确定 Roll 约束)
    #    关键设计：使用 ux_horiz 而不是 ux 来计算叉乘
    #    意义：保证无论 X 轴如何俯仰，Y 轴始终平行于地面
    # ------------------------------------------------------------------
    up_world = np.array([0.0, 0.0, 1.0])
    # 右手定则：Z(上) × X(前) = Y(左)
    uy = np.cross(up_world, ux_horiz)
    norm_y = np.linalg.norm(uy)
    
    # 处理奇异点：理论上 ux_horiz 已处理过，此处为双重保险
    if norm_y < 1e-3:
        uy = np.array([0.0, 1.0, 0.0])
    else:
        uy = uy / norm_y

    # ------------------------------------------------------------------
    # 5. 计算末端 Z 轴 (完成正交基)
    #    Z = X × Y
    #    由于 X 是倾斜的，Y 是水平的，Z 将指向斜上方
    # ------------------------------------------------------------------
    uz = np.cross(ux, uy)
    # 理论上已正交归一，再次归一化以消除累积误差
    uz = uz / np.linalg.norm(uz)
    
    # ------------------------------------------------------------------
    # 6. 构建旋转矩阵 (Rotation Matrix)
    #    列向量形式：R = [X_axis, Y_axis, Z_axis]
    # ------------------------------------------------------------------
    rot_mat = np.eye(3, dtype=np.float64)
    rot_mat[:, 0] = ux  # 第 1 列：X 轴 (接近方向)
    rot_mat[:, 1] = uy  # 第 2 列：Y 轴 (侧向水平)
    rot_mat[:, 2] = uz  # 第 3 列：Z 轴 (法向)
    logger.info(f"In get_target_flange_pose --- rot_mat: {rot_mat}")  
    
    # 转为四元数
    # 注意：需确认 math.matrix_to_quaternion 返回的顺序是 (x,y,z,w) 还是 (w,x,y,z)
    # 常见库如 scipy.spatial.transform.Rotation 使用 (x,y,z,w)
    final_quat = matrix_to_quaternion(rot_mat)
    
    # ------------------------------------------------------------------
    # 7. 计算最终位置 (Position)
    #    沿接近方向 (ux) 反向回退 offset_x 距离
    # ------------------------------------------------------------------
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
