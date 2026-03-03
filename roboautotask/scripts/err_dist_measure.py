'''
Docstring for roboautotask.scripts.err_dist_measure
使用视觉识别实时检测橙子和盘子检测点的距离
'''
import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

# 如果你不能改 config，可在此处直接定义：
TARGET_CLASS_A = "orange"
TARGET_CLASS_B = "plate"
# YOLO_MODEL_PATH = "your_model.pt"

from roboautotask.configs.estimation import YOLO_MODEL_PATH

def get_3d_pts(depth_frame, intr, px, py):
    dist = depth_frame.get_distance(px, py)
    if dist <= 0:
        return None
    x_rs, y_rs, z_rs = rs.rs2_deproject_pixel_to_point(intr, [px, py], dist)
    # 坐标系转换: +X=前, +Y=左, +Z=上
    return np.array([z_rs, -x_rs, -y_rs])

def project_3d_to_2d(intr, point_3d):
    """
    将相机坐标系下的3D点 (X前, Y左, Z上) 投影回像素坐标（用于可视化手动点）
    注意：需逆变换回 RealSense 原始坐标系
    """
    # 逆变换：从 [X_cam, Y_cam, Z_cam] = [Z_rs, -X_rs, -Y_rs]
    # 所以：X_rs = -Y_cam, Y_rs = -Z_cam, Z_rs = X_cam
    X_cam, Y_cam, Z_cam = point_3d
    X_rs = -Y_cam
    Y_rs = -Z_cam
    Z_rs = X_cam

    if Z_rs <= 0:
        return None

    px, py = rs.rs2_project_point_to_pixel(intr, [X_rs, Y_rs, Z_rs])
    return int(px), int(py)


def capture_dual_class_distance(manual_b_point=None):
    """
    模式1（默认）：检测 A 和 B 两类，计算距离。
    模式2（manual_b_point 给定）：只检测 A 类，计算 A 到 manual_b_point 的距离。
    
    manual_b_point: np.array or list of shape (3,), in camera frame [X, Y, Z] (meters)
                    X=forward, Y=left, Z=up
    """
    print(f"Loading YOLO model: {YOLO_MODEL_PATH}...")
    model = YOLO(YOLO_MODEL_PATH)

    names = model.names
    cls_a = [k for k, v in names.items() if v.lower() == TARGET_CLASS_A.lower()]
    if not cls_a:
        raise ValueError(f"类别 A '{TARGET_CLASS_A}' 不在模型中！")
    cls_a = cls_a[0]

    if manual_b_point is not None:
        manual_b_point = np.array(manual_b_point, dtype=np.float32)
        print(f"Manual B point enabled: {manual_b_point} (camera frame, meters)")
        use_manual_b = True
    else:
        cls_b = [k for k, v in names.items() if v.lower() == TARGET_CLASS_B.lower()]
        if not cls_b:
            raise ValueError(f"类别 B '{TARGET_CLASS_B}' 不在模型中！")
        cls_b = cls_b[0]
        use_manual_b = False

    # RealSense 初始化
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)

    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = depth_profile.get_intrinsics()
    align = rs.align(rs.stream.color)

    mode_str = "Manual B Point" if use_manual_b else f"Auto B Class: {TARGET_CLASS_B}"
    print(f"Mode: {mode_str}")
    print(f"Detecting A: '{TARGET_CLASS_A}' (green). Press 'q' to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            results = model(img, verbose=False)[0]
            boxes = results.boxes

            obj_a = None  # (cx, cy, pt3d, x1, y1, x2, y2, conf)

            # 只检测 A 类
            for b in boxes:
                cls_id = int(b.cls)
                conf = float(b.conf)
                x1, y1, x2, y2 = map(int, b.xyxy[0].cpu().numpy())
                cx, cy = int((x1 + x2) // 2), int((y1 + y2) // 2)
                pt3d = get_3d_pts(depth_frame, intr, cx, cy)

                if cls_id == cls_a:
                    if obj_a is None or conf > obj_a[-1]:
                        obj_a = (cx, cy, pt3d, x1, y1, x2, y2, conf)

            # 绘制 A 类
            if obj_a and obj_a[2] is not None:
                cx, cy, pt3d, x1, y1, x2, y2, _ = obj_a
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{TARGET_CLASS_A}: ({pt3d[0]:.2f}, {pt3d[1]:.2f}, {pt3d[2]:.2f})"
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 处理 B 点
            b_point_3d = None
            b_point_2d = None

            if use_manual_b:
                b_point_3d = manual_b_point
                # 投影到图像（用于可视化）
                proj = project_3d_to_2d(intr, b_point_3d)
                if proj:
                    b_point_2d = proj
                    # 画一个十字标记
                    x, y = b_point_2d
                    cv2.drawMarker(img, (x, y), (255, 0, 255), markerType=cv2.MARKER_CROSS,
                                   markerSize=20, thickness=2)
                    cv2.putText(img, "Manual Target", (x - 30, y - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            else:
                # 自动检测 B 类
                obj_b = None
                for b in boxes:
                    cls_id = int(b.cls)
                    conf = float(b.conf)
                    x1, y1, x2, y2 = map(int, b.xyxy[0].cpu().numpy())
                    cx, cy = int((x1 + x2) // 2), int((y1 + y2) // 2)
                    pt3d = get_3d_pts(depth_frame, intr, cx, cy)

                    if cls_id == cls_b:
                        if obj_b is None or conf > obj_b[-1]:
                            obj_b = (cx, cy, pt3d, x1, y1, x2, y2, conf)

                if obj_b and obj_b[2] is not None:
                    cx, cy, pt3d, x1, y1, x2, y2, _ = obj_b
                    b_point_3d = pt3d
                    b_point_2d = (cx, cy)
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    label = f"{TARGET_CLASS_B}: ({pt3d[0]:.6f}, {pt3d[1]:.6f}, {pt3d[2]:.6f})"
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # 计算距离
            distance_text = "Distance: N/A"
            if obj_a and obj_a[2] is not None and b_point_3d is not None:
                dist_m = np.linalg.norm(obj_a[2] - b_point_3d)
                distance_text = f"Distance: {dist_m:.3f} m"

                if b_point_2d is not None:
                    cv2.line(img, (obj_a[0], obj_a[1]), b_point_2d, (255, 0, 255), 2)
                    mid_x = (obj_a[0] + b_point_2d[0]) // 2
                    mid_y = (obj_a[1] + b_point_2d[1]) // 2
                    cv2.putText(img, f"{dist_m:.3f}m", (mid_x, mid_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

            cv2.putText(img, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("A-to-B Distance Measurement", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


# ======================
# 使用示例
# ======================
def main():
    # 示例1：自动检测两个类别
    capture_dual_class_distance()

    # 示例2：手动指定B点（单位：米，相机坐标系）
    # 假设你想测量物体到 (0.4m 前, 0.0 左右, 0.3m 高) 的距离
    manual_target = [0.647000,0.215322,0.043712]  # [X=前, Y=左, Z=上]
    #capture_dual_class_distance(manual_b_point=manual_target)

if __name__ == "__main__":
    main()