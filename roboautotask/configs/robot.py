from roboautotask.configs import topic

# 机械臂配置字典
ARM_CONFIG = {
    'left': {
        'ee_pub': topic.LEFT_EE_PUB,
        'ee_sub': topic.LEFT_EE_SUB,
        'gripper_pub': topic.LEFT_GRIPPER_PUB,
        'gripper_sub': topic.LEFT_GRIPPER_SUB,
        'home_pose': {
            'pos': [0.158994, 0.335901, 0.453466],
            'quat': [0.0038, 0.0018, 0.0058, 0.99997],
            'gripper': 100.0
        }
    },
    'right': {
        'ee_pub': topic.RIGHT_EE_PUB,
        'ee_sub': topic.RIGHT_EE_SUB,
        'gripper_pub': topic.RIGHT_GRIPPER_PUB,
        'gripper_sub': topic.RIGHT_GRIPPER_SUB,
        'home_pose': {
            'pos': [0.158795, -0.337121, 0.4562],
            'quat': [0.010879, -0.010736, -0.009139, 0.999845],
            'gripper': 100.0
        }
    }
}


def get_arm_home_pose(arm: str = 'left'):
    """返回指定手臂的 Home 位姿 (pos, quat)。"""
    cfg = ARM_CONFIG[arm]['home_pose']
    return cfg['pos'], cfg['quat']


# ================= 向后兼容别名（以左臂 Home 为准）=================
ROBOT_START_POS, ROBOT_START_ORI = get_arm_home_pose('left')

# 运动控制参数
CONTROL_PARAMS = {
    'steps': 50,                # 插值步数
    'lift_height': 0.07,        # 抬升高度 (m)
    'stabilize_threshold': 0.02, # 到位判断阈值 (m)
    'stabilize_timeout': 3.0,   # 到位等待超时 (s)
    'grip_duration': 1.0        # 夹爪动作时间 (s)
}