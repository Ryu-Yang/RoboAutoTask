import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

from roboautotask.configs.estimation import TARGET_CLASS, YOLO_MODEL_PATH, CONFIDENCE_FRAMES

def get_3d_pts(depth_frame, intr, px, py):
    dist = depth_frame.get_distance(px, py)
    if dist <= 0:
        return None
    x_rs, y_rs, z_rs = rs.rs2_deproject_pixel_to_point(intr, [px, py], dist)
    # 坐标系转换: +X=前, +Y=左, +Z=上
    return np.array([z_rs, -x_rs, -y_rs])

def capture_target_coordinate(TARGET_CLASS=TARGET_CLASS):
    """
    运行相机循环，检测到稳定目标后返回 相机坐标系下的(x,y,z)
    """
    print(f"Loading YOLO model: {YOLO_MODEL_PATH}...")
    model = YOLO(YOLO_MODEL_PATH)
    
    # 检查类别
    names = model.names
    cls_target = [k for k, v in names.items() if v.lower() == TARGET_CLASS.lower()]
    if not cls_target:
        raise ValueError(f"类别 '{TARGET_CLASS}' 不在模型中！")
    cls_target = cls_target[0]

    # RealSense 初始化
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    
    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = depth_profile.get_intrinsics()
    align = rs.align(rs.stream.color)

    collected_points = []
    final_point = None

    print(f"Looking for '{TARGET_CLASS}'... Please keep camera steady.")

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
            
            current_frame_point = None

            for b in boxes:
                if int(b.cls) == cls_target:
                    x1, y1, x2, y2 = map(int, b.xyxy[0].cpu().numpy())
                    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                    # --- 新增：计算真实世界中的长和宽 ---
                    dist = depth_frame.get_distance(cx, cy)
                    if dist > 0:
                        # 得到左上角点和右下角点的 3D 坐标
                        p_topleft = rs.rs2_deproject_pixel_to_point(intr, [x1, y1], dist)
                        p_bottomright = rs.rs2_deproject_pixel_to_point(intr, [x2, y2], dist)
                        
                        # 真实世界的宽度和高度 (单位是米)
                        real_w = abs(p_bottomright[0] - p_topleft[0])
                        real_h = abs(p_bottomright[1] - p_topleft[1])

                        line = f"{real_w} {real_h}"
                        # 写入文件（覆盖模式）
                        with open('palced_obj_size.txt', 'w') as f:
                            f.write(line)
                    # ----------------------------------
                    
                    # 获取3D点
                    pt_3d = get_3d_pts(depth_frame, intr, cx, cy)
                    
                    # 绘制
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    if pt_3d is not None:
                        current_frame_point = pt_3d
                        label = f"{pt_3d[0]:.2f},{pt_3d[1]:.2f},{pt_3d[2]:.2f}"
                        cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 稳定性检测逻辑
            if current_frame_point is not None:
                collected_points.append(current_frame_point)
            else:
                collected_points = [] # 丢失目标，重置

            # 显示状态
            cv2.putText(img, f"Collecting: {len(collected_points)}/{CONFIDENCE_FRAMES}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Vision Sensor", img)

            # 如果收集了足够的帧，取平均值并退出
            if len(collected_points) >= CONFIDENCE_FRAMES:
                avg_pt = np.mean(collected_points, axis=0)
                print(f"Target Locked at (Cam Frame): {avg_pt}")
                final_point = avg_pt
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("User cancelled.")
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return final_point