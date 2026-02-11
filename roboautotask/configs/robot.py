from roboautotask.configs import topic
# ================= 机械臂运动默认位姿参数 =================
ROBOT_START_POS = [0.05899396416917888, 0.3359014578850215, 0.3334655269659172]
ROBOT_START_ORI = [0.0038, 0.0018, 0.0058, 0.99997]

# 运动控制参数
CONTROL_PARAMS = {
    'steps': 50,                # 插值步数
    'lift_height': 0.07,        # 抬升高度 (m)
    'stabilize_threshold': 0.02, # 到位判断阈值 (m)
    'stabilize_timeout': 3.0,   # 到位等待超时 (s)
    'grip_duration': 1.0        # 夹爪动作时间 (s)
}

# 机械臂配置字典
ARM_CONFIG = {
    'left': {
        'ee_pub': topic.LEFT_EE_PUB,
        'ee_sub': topic.LEFT_EE_SUB,
        'gripper_pub': topic.LEFT_GRIPPER_PUB,
        'gripper_sub': topic.LEFT_GRIPPER_SUB,
        # 左臂空闲时的默认停靠/保持姿态 (基于统一基座标系)
        # 如果没有特定Home点，驱动初始化时会读取当前位置作为保持点
        'home_pose': {
            'pos': [0.058994, 0.335901, 0.333466],
            'quat': [0.0038, 0.0018, 0.0058, 0.99997],
            'gripper': 100.0
        }
    },
    'right': {
        'ee_pub': topic.RIGHT_EE_PUB,
        'ee_sub': topic.RIGHT_EE_SUB,
        'gripper_pub': topic.RIGHT_GRIPPER_PUB,
        'gripper_sub': topic.RIGHT_GRIPPER_SUB,
        # 右臂空闲时的默认保持姿态 (示例数据，请替换为实际值)
        'home_pose': {
            'pos': [0.058795, -0.337121, 0.337762],
            'quat': [0.010879, -0.010736, -0.009139, 0.999845],
            'gripper': 100.0
        }
    }
}