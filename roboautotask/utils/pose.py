import numpy as np
import os

def save_pose_to_file(filepath, position, quaternion):
    """
    将位置 (3,) 和四元数 (4,) 保存为一行文本，格式: [x y z] [qx qy qz qw]
    
    参数:
        filepath (str): 文件路径
        position (list or np.ndarray): 长度为3的位置数组
        quaternion (list or np.ndarray): 长度为4的四元数数组 [x, y, z, w]
    """
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)

    if position.shape != (3,):
        raise ValueError(f"Position must be length 3, got {position.shape}")
    if quaternion.shape != (4,):
        raise ValueError(f"Quaternion must be length 4, got {quaternion.shape}")

    # 格式化为字符串，保留足够精度（避免科学计数法）
    pos_str = ' '.join(f"{x:.6f}".rstrip('0').rstrip('.') for x in position)
    quat_str = ' '.join(f"{x:.6f}".rstrip('0').rstrip('.') for x in quaternion)

    line = f"[{pos_str}] [{quat_str}]"

    # 写入文件（覆盖模式）
    with open(filepath, 'w') as f:
        f.write(line)


def load_pose_from_file(filepath):
    """
    从文件中读取一行位姿数据，返回 position 和 quaternion 的 NumPy 数组
    
    参数:
        filepath (str): 文件路径
    
    返回:
        tuple: (position: np.ndarray (3,), quaternion: np.ndarray (4,))
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    with open(filepath, 'r') as f:
        line = f.read().strip()

    if not line:
        raise ValueError("File is empty")

    # 拆分两个方括号部分
    if line.count('[') != 2 or line.count(']') != 2:
        raise ValueError("Invalid format: expected two bracket groups")

    try:
        parts = line.split('] [')
        if len(parts) != 2:
            raise ValueError("Expected exactly two groups in brackets")

        part1 = parts[0].lstrip('[')
        part2 = parts[1].rstrip(']')

        pos = np.array([float(x) for x in part1.split()], dtype=np.float64)
        quat = np.array([float(x) for x in part2.split()], dtype=np.float64)

        if pos.shape != (3,) or quat.shape != (4,):
            raise ValueError("Position must have 3 values and quaternion 4 values")

        return pos, quat

    except Exception as e:
        raise ValueError(f"Failed to parse pose file '{filepath}': {e}")