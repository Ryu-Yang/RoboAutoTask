"""
记录机械臂末端位姿 + ArUco 标记位姿
按键：r 记录，q 退出
ubuntu自带系统
"""
import rclpy
import threading
import sys
import select
import datetime
import cv2
import numpy as np
import pyrealsense2 as rs
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from roboautotask.configs.svd import T_tool_tip
from roboautotask.configs.topic import LEFT_EE_SUB
from roboautotask.utils.math import pose_to_matrix, matrix_to_pose


class RecordArmArucoNode(Node):
    def __init__(self, csv_path):
        super().__init__('record_arm_aruco_node')

        # ----------------  RealSense  ----------------
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        try:
            prof = self.pipeline.start(cfg)
            self.align = rs.align(rs.stream.color)
            intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.intrinsics = intr
            self.get_logger().info('RealSense D455 initialized')
        except Exception as e:
            self.get_logger().error(f'Realsense init failed: {e}')
            self.pipeline = None
            self.align = None
            self.intrinsics = None

        # ----------------  ArUco  ----------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        # self.aruco_params = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ----------------  线程同步对象  ----------------
        self.frames_lock = threading.Lock()
        self.latest_color = None          # 带标注的图像
        self.latest_aruco = {}            # {id: (x,y,z)}

        # ----------------  提示标志  ----------------
        self.show_recorded_flag = False
        self.recorded_flag_time = 0.0     # 秒

        # ----------------  ROS 订阅  ----------------
        self.create_subscription(PoseStamped,
                                LEFT_EE_SUB,
                                self.pose_cb, 10)
        self.latest_pose = None
        self.latest_ori = None
        self.pose_lock = threading.Lock()

        # ----------------  文件  ----------------
        self.csv_path = csv_path
        with open(self.csv_path, 'w', encoding='utf-8') as f:
            f.write('timestamp,arm_x,arm_y,arm_z,num_aruco,aruco_data\n')
            f.write('# Format: ts,arm_x,arm_y,arm_z,n,id1,x1,y1,z1,id2,...\n')

        # ----------------  启动线程  ----------------
        if self.pipeline:
            self.cam_thread = threading.Thread(target=self.camera_thread_func, daemon=True)
            self.cam_thread.start()
        self.preview_running = True
        self.preview_thread = threading.Thread(target=self.preview_thread_func, daemon=True)
        self.preview_thread.start()

    # ------------------------------------------------------------------
    #                             回调
    # ------------------------------------------------------------------
    def pose_cb(self, msg: PoseStamped):
        with self.pose_lock:
            self.latest_pose = msg.pose.position
            self.latest_ori = msg.pose.orientation

    # ------------------------------------------------------------------
    #                         相机采集线程
    # ------------------------------------------------------------------
    def camera_thread_func(self):
        while rclpy.ok() and self.pipeline:
            try:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                depth_f = aligned.get_depth_frame()
                color_f = aligned.get_color_frame()
                if not depth_f or not color_f:
                    continue
                color_np = np.asanyarray(color_f.get_data())
                display = color_np.copy()

                # ArUco 检测
                corners, ids, _ = self.detector.detectMarkers(color_np)
                markers = {}
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(display, corners, ids)
                    for i, corner in enumerate(corners):
                        pts_3d = []
                        valid = True
                        for p in corner[0]:
                            x_px, y_px = int(p[0]), int(p[1])
                            if not (0 <= x_px < 640 and 0 <= y_px < 480):
                                valid = False; break
                            d = depth_f.get_distance(x_px, y_px)
                            if d <= 0 or d > 5.0:
                                valid = False; break
                            pt3 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x_px, y_px], d)
                            pts_3d.append(pt3)
                        if not valid or len(pts_3d) != 4:
                            continue
                        # 坐标系：+X前 +Y左 +Z上
                        center_rs = np.mean(pts_3d, axis=0)
                        x, y, z = center_rs[2], -center_rs[0], -center_rs[1]
                        mid = int(ids[i][0])
                        markers[mid] = (x, y, z)
                        # 画文字
                        cx = int(np.mean(corner[0][:, 0]))
                        cy = int(np.mean(corner[0][:, 1]))
                        txt = f'ID{mid}:{x:.2f},{y:.2f},{z:.2f}'
                        (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                        cv2.rectangle(display, (cx-5, cy-25), (cx+w+5, cy), (0,0,0), -1)
                        cv2.putText(display, txt, (cx, cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

                # 状态条
                status = f"Press 'r' to record | 'q' to quit | Detected: {len(markers)} markers"
                cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                with self.frames_lock:
                    self.latest_color = display
                    self.latest_aruco = markers.copy()
            except Exception as e:
                self.get_logger().error(f'camera thread: {e}')

    # ------------------------------------------------------------------
    #                         预览显示线程
    # ------------------------------------------------------------------
    def preview_thread_func(self):
        win = 'ArUco Detection Preview'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 800, 600)
        while rclpy.ok() and self.preview_running:
            with self.frames_lock:
                img = self.latest_color.copy() if self.latest_color is not None else \
                      np.zeros((480, 640, 3), np.uint8)

            # 画“RECORDED!”提示
            if self.show_recorded_flag:
                now = cv2.getTickCount() / cv2.getTickFrequency()
                if now - self.recorded_flag_time < 0.5:
                    cv2.putText(img, 'RECORDED!', (250, 100),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
                else:
                    self.show_recorded_flag = False

            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                self.preview_running = False
        cv2.destroyWindow(win)

    # ------------------------------------------------------------------
    #                           记录函数
    # ------------------------------------------------------------------
    def record_once(self):
        with self.pose_lock:
            if self.latest_pose is None:
                self.get_logger().warn('No arm pose received yet')
                return
            arm = self.latest_pose
            arm_ori = self.latest_ori
        with self.frames_lock:
            markers = self.latest_aruco.copy()

        ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        my_pos = [arm.x, arm.y, arm.z]
        my_quat = [arm_ori.x, arm_ori.y, arm_ori.z, arm_ori.w]  # 绕Y轴旋转约90度的四元数 (x,y,z,w)
        T = pose_to_matrix(my_pos, my_quat)
        # 弃用自定义变换矩阵，改用config统一矩阵
        # T_tool_tip = np.array([
        #     [1, 0, 0, 0.051],
        #     [0, 1, 0, 0],
        #     [0, 0, 1, 0],
        #     [0, 0, 0, 1]
        # ])
        T_base_tip = T @ T_tool_tip
        new_pos,new_ori = matrix_to_pose(T_base_tip)

        arm_str = f'{new_pos[0].item():.6f},{new_pos[1].item():.6f},{new_pos[2].item():.6f}'
        if markers:
            sorted_ids = sorted(markers.keys())
            aruco_str = f'{len(markers)}' + ''.join(f',{i},{markers[i][0]:.6f},{markers[i][1]:.6f},{markers[i][2]:.6f}'
                                                     for i in sorted_ids)
        else:
            aruco_str = '0'
        line = f'{ts},{arm_str},{aruco_str}\n'
        with open(self.csv_path, 'a', encoding='utf-8') as f:
            f.write(line)
        self.get_logger().info(f'Recorded: arm ({new_pos[0].item():.3f}, {new_pos[1].item():.3f}, {new_pos[2].item():.3f})  {len(markers)} marker(s)')

        # 触发提示
        self.show_recorded_flag = True
        self.recorded_flag_time = cv2.getTickCount() / cv2.getTickFrequency()

    # ------------------------------------------------------------------
    #                           生命周期
    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        self.preview_running = False
        if self.pipeline:
            self.pipeline.stop()
        try:
            cv2.destroyWindow('ArUco Detection Preview')
        except:
            pass
        super().destroy_node()


# ----------------------------------------------------------------------
#                           键盘监听（主线程）
# ----------------------------------------------------------------------
def keyboard_loop(node: RecordArmArucoNode):
    print('\n' + '='*60)
    print('  CONTROL:  r = record    q = quit')
    print('='*60 + '\n')
    while rclpy.ok():
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1).lower()
            if key == 'r':
                node.record_once()
            elif key == 'q':
                print('Quit requested.')
                rclpy.shutdown()
                break


# ----------------------------------------------------------------------
#                                main
# ----------------------------------------------------------------------
def main():
    rclpy.init()
    out_dir = Path('output')
    out_dir.mkdir(exist_ok=True)
    csv_path = out_dir / f'arm_aruco_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.csv'
    node = RecordArmArucoNode(csv_path)
    print(f'Saving to: {csv_path.absolute()}')

    # 把键盘监听放在主线程，spin 用另一个线程
    thread_spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread_spin.start()
    try:
        keyboard_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread_spin.join(timeout=1)
        print(f'\nAll data saved -> {csv_path.absolute()}')


if __name__ == '__main__':
    main()