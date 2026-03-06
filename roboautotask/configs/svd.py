import numpy as np

# ================= 标定参数 (SVD) =================
# Robot = R @ Cam + t

# ---------- 左手 (left arm) ----------
CALIB_R_LEFT = np.array([
    [ 0.7154715,  -0.00816792,  0.69859417],
    [ 0.00434896,  0.99996435,  0.0072375 ],
    [-0.69862839, -0.00214006,  0.71548152]
])
CALIB_T_LEFT = np.array([ 0.04316503, -0.01616264,  0.65823296])

# ee与夹爪标定点的工具变换矩阵（左手）
T_tool_tip_LEFT = np.array([
            [1, 0, 0, 0.051],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

# ---------- 右手 (right arm) ----------
# TODO: 请替换为实际右手标定结果
CALIB_R_RIGHT = np.array([
    [ 0.7154715,  -0.00816792,  0.69859417],
    [ 0.00434896,  0.99996435,  0.0072375 ],
    [-0.69862839, -0.00214006,  0.71548152]
])
CALIB_T_RIGHT = np.array([ 0.04316503, -0.01616264,  0.65823296])

# ee与夹爪标定点的工具变换矩阵（右手）
# TODO: 请替换为实际右手工具偏移
T_tool_tip_RIGHT = np.array([
            [1, 0, 0, 0.051],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

# ---------- 向后兼容别名（默认左手）----------
CALIB_R = CALIB_R_LEFT
CALIB_T = CALIB_T_LEFT
T_tool_tip = T_tool_tip_LEFT


def get_calib_params(arm: str = 'left'):
    """
    根据手臂选择返回对应的标定参数。

    Args:
        arm: 'left' 或 'right'

    Returns:
        (CALIB_R, CALIB_T, T_tool_tip)
    """
    arm = arm.lower()
    if arm == 'right':
        return CALIB_R_RIGHT, CALIB_T_RIGHT, T_tool_tip_RIGHT
    elif arm == 'left':
        return CALIB_R_LEFT, CALIB_T_LEFT, T_tool_tip_LEFT
    else:
        raise ValueError(f"Unknown arm '{arm}', must be 'left' or 'right'")
