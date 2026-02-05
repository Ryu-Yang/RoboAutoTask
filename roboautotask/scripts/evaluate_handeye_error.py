#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_aruco.py  ——  在 D455 彩色图上检测 ArUco 并打印/显示 3D 中心坐标
新增：实时读取 /relaxed_ik/motion_control/pose_ee_arm_left 并在窗口左上角显示机械臂末端位置
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from roboautotask.utils.math import pose_to_matrix, matrix_to_pose
from roboautotask.configs.svd import T_tool_tip

# ---------------------------
# ROS2 节点：只订阅，拿到最新 Pose 后保存在 self.latest_xyz
# ---------------------------
class ArmPoseSubscriber(Node):
    def __init__(self):
        super().__init__('arm_pose_listener')
        self.latest_xyz = (0.0, 0.0, 0.0)          # 默认
        self.latest_ori = (0.0, 0.0, 0.0, 1.0)
        self.sub = self.create_subscription(
            PoseStamped,
            '/relaxed_ik/motion_control/pose_ee_arm_left',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.latest_xyz = (p.x, p.y, p.z)
        self.latest_ori = (o.x, o.y, o.z, o.w)

# ---------------------------
# 1. 初始化 RealSense
# ---------------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()


dis_b = [0,0,0]

def dist3d(p, q):
    """两点三维欧氏距离，p、q 均可迭代 (x,y,z)"""
    return sum((pi - qi) ** 2 for pi, qi in zip(p, q)) ** 0.5


# ---------------------------
# 2. ArUco 检测器
# ---------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# ---------------------------
# 3. 初始化 ROS2
# ---------------------------
rclpy.init()
arm_node = ArmPoseSubscriber()

# ---------------------------
# 4. 主循环
# ---------------------------
try:
    while True:
        # 4.1 旋转 ROS2 回调一次，拿最新 pose
        rclpy.spin_once(arm_node, timeout_sec=0.001)

        # 4.2 取帧
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # 4.3 ArUco 检测与 3D 坐标计算（原逻辑不变）
        corners, ids, _ = detector.detectMarkers(color_image)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
            for i, corner in enumerate(corners):
                pts = corner[0]
                points_3d = []
                valid = True
                for pt in pts:
                    x_px, y_px = int(pt[0]), int(pt[1])
                    if not (0 <= x_px < 640 and 0 <= y_px < 480):
                        valid = False
                        break
                    depth = depth_frame.get_distance(x_px, y_px)
                    if depth <= 0 or depth > 5.0:
                        valid = False
                        break
                    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x_px, y_px], depth)
                    points_3d.append(point_3d)

                if not valid or len(points_3d) != 4:
                    continue

                center_rs = np.mean(points_3d, axis=0)
                x = center_rs[2]
                y = -center_rs[0]
                z = -center_rs[1]
                marker_id = ids[i][0]

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                coord_text = f"ID{marker_id}: X={x:.6f}, Y={y:.6f}, Z={z:.6f}"
                dis_b[0]=(float(x))
                dis_b[1]=(float(y))
                dis_b[2]=(float(z))
                print(coord_text)

                (text_w, text_h), _ = cv2.getTextSize(coord_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(color_image, (cx - 10, cy - 30), (cx + text_w + 10, cy - 5), (0, 0, 0), -1)
                cv2.putText(color_image, coord_text, (cx, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # 4.4 在窗口左上角显示机械臂末端位置
        x, y, z = arm_node.latest_xyz
        R = np.array([
            [ 0.7154715,   0.00434896, -0.69862839],
            [-0.00816792,  0.99996435, -0.00214006],
            [ 0.69859417,  0.0072375,   0.71548152]
        ])
        t = np.array([0.42904718,  0.01792329, -0.50099138])
##########################
        my_pos = [x, y, z]
        my_quat = [arm_node.latest_ori[0], arm_node.latest_ori[1], arm_node.latest_ori[2], arm_node.latest_ori[3]]  # 绕Y轴旋转约90度的四元数 (x,y,z,w)
        T = pose_to_matrix(my_pos, my_quat)
        # T_tool_tip = np.array([
        #     [1, 0, 0, 0.051],
        #     [0, 1, 0, 0],
        #     [0, 0, 1, 0],
        #     [0, 0, 0, 1]
        # ])
        T_base_tip = T @ T_tool_tip
        new_pos,new_ori = matrix_to_pose(T_base_tip)

        # arm_str = f'{new_pos[0].item():.6f},{new_pos[1].item():.6f},{new_pos[2].item():.6f}'
######################################
        P_old = new_pos
        P_new = R @ P_old + t
        #arm_text = f"Arm: X={x:.3f}  Y={y:.3f}  Z={z:.3f}"
        dis_a = P_new.tolist()
        dis = dist3d(dis_a,dis_b)
        arm_posi = str(dis)
        cv2.putText(color_image, arm_posi, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(color_image, str(dis_a), (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(color_image, str(dis_b), (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # 4.5 显示
        cv2.imshow('ArUco 3D Detection (D455)', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    arm_node.destroy_node()
    rclpy.shutdown()