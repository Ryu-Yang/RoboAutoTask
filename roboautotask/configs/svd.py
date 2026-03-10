import numpy as np

# ================= 标定参数 (SVD) =================
# Robot = R @ Cam + t

# ---------- 左手 (left arm) ----------
CALIB_R_LEFT = np.array(
    [
        [ 0.56175814, -0.08168945,  0.82325854],
        [ 0.00633508,  0.99550864,  0.0944585 ],
        [-0.82727726, -0.04784742,  0.55975259]
    ]
)
CALIB_T_LEFT = np.array(
    [0.0170217, -0.27692587, 0.6300922]
)

# ee与夹爪标定点的工具变换矩阵（左手）
T_tool_tip_LEFT = np.array(
    [
        [1, 0, 0, 0.107],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
)

# ---------- 右手 (right arm) ----------
# TODO: 请替换为实际右手标定结果
CALIB_R_RIGHT = np.array(
    [
        [ 0.47149909,  0.03549139,  0.88115207],
        [-0.13733244,  0.98995456,  0.03361196],
        [-0.87110758, -0.13685877,  0.4716368 ]
    ]
)
CALIB_T_RIGHT = np.array(
    [0.1074867, 0.02831579, 0.60470831]
)

# ee与夹爪标定点的工具变换矩阵（右手
T_tool_tip_RIGHT = np.array(
    [
        [1, 0, 0, 0.107],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
)

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
