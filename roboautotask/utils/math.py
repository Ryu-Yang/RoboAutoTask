import numpy as np
from typing import Tuple, List
import math


def matrix_to_quaternion(rot_mat):
    """
    将 3x3 旋转矩阵转换为四元数 (x, y, z, w)
    """
    # 基于 Shepperd 方法 (数值稳定版本)
    m = rot_mat
    trace = np.trace(m)
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    
    return np.array([x, y, z, w])

def pose_to_matrix(pos, quat):
    """
    将位置(x,y,z)和四元数(x,y,z,w)转换为4x4变换矩阵
    """
    qx, qy, qz, qw = quat
    # 计算旋转矩阵的各项分量
    # 这里的数学公式是经典的四元数转旋转矩阵公式
    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx*qy - qz*qw)
    r02 = 2 * (qx*qz + qy*qw)
    
    r10 = 2 * (qx*qy + qz*qw)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy*qz - qx*qw)
    
    r20 = 2 * (qx*qz - qy*qw)
    r21 = 2 * (qy*qz + qx*qw)
    r22 = 1 - 2 * (qx**2 + qy**2)
    
    # 组装成 4x4 矩阵
    matrix = np.array([
        [r00, r01, r02, pos[0]],
        [r10, r11, r12, pos[1]],
        [r20, r21, r22, pos[2]],
        [0,   0,   0,   1]
    ])
    return matrix

def matrix_to_pose(matrix):
    """
    将4x4变换矩阵拆解为位置(x,y,z)和四元数(x,y,z,w)
    """
    # 提取位置
    pos = matrix[:3, 3]
    
    # 提取旋转矩阵并计算四元数 (Shepperd's Algorithm 简化版)
    r = matrix[:3, :3]
    tr = np.trace(r)
    
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        qw = 0.25 / s
        qx = (r[2, 1] - r[1, 2]) * s
        qy = (r[0, 2] - r[2, 0]) * s
        qz = (r[1, 0] - r[0, 1]) * s
    else:
        # 如果轨迹小于等于0，需要找对角线最大的元素
        if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
            
    return pos, np.array([qx, qy, qz, qw])

def invert_rt(R: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    输入R, t求逆
    Parameters
    ----------
    R : (3,3) array_like   rotation matrix
    t : (3,)  or (3,1) array_like   translation vector

    Returns
    -------
    R_inv : (3,3) ndarray   inverse rotation (= R.T)
    t_inv : (3,)  ndarray   inverse translation (= -R.T @ t)
    """
    R = np.asarray(R, dtype=float)
    t = np.asarray(t, dtype=float).reshape(3)

    if R.shape != (3, 3):
        raise ValueError("R must be 3×3")
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        raise ValueError("R is not a valid rotation matrix")

    R_inv = R.T                       # 旋转矩阵正交：逆=转置
    t_inv = -R_inv @ t                # 公式推导见下方
    return R_inv, t_inv

# ================= 辅助函数 =================
def quaternion_slerp(q0, q1, t):
    """ 四元数球面线性插值 """
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = np.dot(q0, q1)

    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    
    return s0 * q0 + s1 * q1


def generate_random_points_around_center(
    center_point: List[float],  # 输入的中心点 [x, y, z]
    rectangle_width: float = 0.19,  # 矩形区域宽度（x方向）
    rectangle_length: float = 0.4,  # 矩形区域长度（y方向）
    rectangle_height: float = 0.01,   # 矩形区域高度（z方向）
    exclusion_radius: float = 0.095,   # 排除圆的半径
    num_points: int = 1,           # 要生成的点的数量
    distribution_power: float = 2.0   # 控制概率分布的参数，值越大越靠近中心
) -> np.ndarray:
    """
    在以输入点为中心的矩形区域内生成随机点
    
    参数:
        center_point: 中心点坐标 [x, y, z]
        rectangle_width: 矩形宽度（x方向范围）
        rectangle_length: 矩形长度（y方向范围）  
        rectangle_height: 矩形高度（z方向范围）
        exclusion_radius: 排除圆的半径
        num_points: 要生成的点的数量
        distribution_power: 控制点靠近中心的概率分布（值越大越靠近中心）
    
    返回:
        np.ndarray: 形状为 (num_points, 3) 的随机点数组
    """
    
    # 确保中心点是一个长度为3的列表/数组
    center = np.array(center_point, dtype=float)
    if len(center) != 3:
        raise ValueError("中心点必须是包含x, y, z三个值的数组")
    
    # 将矩形区域分成多个环形区域，实现不同概率
    num_rings = 5  # 将区域分成5个环形区域
    points_generated = 0
    all_points = []
    
    while points_generated < num_points:
        # 在矩形区域内生成均匀分布的点
        rand_point = np.array([
            center[0] + np.random.uniform(-rectangle_width/2, rectangle_width/2),
            center[1] + np.random.uniform(-rectangle_length/2, rectangle_length/2),
            center[2] + np.random.uniform(-rectangle_height/2, rectangle_height/2)
        ])
        
        # 计算到中心点在xy平面的距离
        xy_distance = np.sqrt((rand_point[0] - center[0])**2 + 
                             (rand_point[1] - center[1])**2)
        
        # 排除圆内的点
        if xy_distance <= exclusion_radius:
            continue
        
        # 根据距离计算接受概率（距离越小，接受概率越高）
        # 使用指数衰减函数计算概率
        max_distance = np.sqrt((rectangle_width/2)**2 + (rectangle_length/2)**2)
        normalized_distance = xy_distance / max_distance
        
        # 接受概率随距离增加而增加，使用幂函数控制
        # acceptance_prob = (1 - normalized_distance) ** distribution_power
        acceptance_prob = normalized_distance ** distribution_power
        
        # 随机决定是否接受这个点
        if np.random.random() < acceptance_prob:
            all_points.append(rand_point)
            points_generated += 1
    
    return np.array(all_points)


def obj_is_in_placement(point,center,width,height):
    """
    判断点是否在矩形内
    :param point: (x1, y1)
    :param center: (xc, yc) 矩形中心
    :param width: 宽度
    :param height: 高度
    :return: Boolean
    """
    x1, y1, z1 = point
    xc, yc, zc = center
    
    # 计算水平和垂直方向的半长
    half_w = width / 2
    half_h = height / 2
    
    # 检查 x 和 y 是否都在边界范围内
    in_x = xc - half_w <= x1 <= xc + half_w
    in_y = yc - half_h <= y1 <= yc + half_h
    
    return in_x and in_y