
import logging_mp
import cv2
import numpy as np
from ultralytics import YOLO
import time
from collections import deque

from roboautotask.configs.estimation import TARGET_CLASS, YOLO_MODEL_PATH, CONFIDENCE_FRAMES
from roboautotask.camera.realsense import RealsenseCameraClientNode


logger = logging_mp.get_logger(__name__)


class TargetDetection():
    """
    使用目标检测（按需检测版本）
    """

    def __init__(
            self, 
            target_class=TARGET_CLASS,
            yolo_model_path=YOLO_MODEL_PATH,
            confidence_frames=CONFIDENCE_FRAMES,
        ):
        # 参数
        self.target_class = target_class
        self.confidence_frames = confidence_frames
        
        # 初始化YOLO模型
        logger.info(f"Loading YOLO model: {yolo_model_path}...")
        self.model = YOLO(yolo_model_path)
        logger.info(f"Load YOLO success")
        
        # # 检查类别
        # names = self.model.names
        # cls_target = [k for k, v in names.items() if v.lower() == target_class.lower()]
        # if not cls_target:
        #     raise ValueError(f"类别 '{target_class}' 不在模型中！")
        # self.cls_target = cls_target[0]
        # logger.info(f"Target class: {target_class} (ID: {self.cls_target})")
        
        # 状态变量
        self.collected_points = deque(maxlen=confidence_frames * 2)
        self.detection_running = False
        self.camera = None
        
        logger.info("Target detection initialized (on-demand mode)")
    
    def get_3d_pts(self, x, y, depth_image=None):
        """
        获取3D坐标点（基于深度相机内参）
        
        Args:
            x, y: 像素坐标（深度图像坐标系）
            depth_image: 深度图像（可选）
            
        Returns:
            np.array: 3D坐标 [x, y, z]
                     转换后坐标系: +X=前, +Y=左, +Z=上
        """
        try:
            # 方法1：直接使用深度像素坐标
            # 注意：这里的x,y应该是深度图像上的坐标
            # 对于对齐深度，彩色图像坐标和深度图像坐标一致
            point_3d = self.camera.pixel_to_3d(x, y, depth_image, use_depth_intrinsics=True)
            
            if point_3d is None:
                return None
            
            # 获取深度相机坐标系下的点
            # 深度相机坐标系: +X=右, +Y=下, +Z=前
            x_cam, y_cam, z_cam = point_3d
            
            # 转换到目标坐标系: +X=前, +Y=左, +Z=上
            x_target = z_cam       # 前
            y_target = -x_cam      # 左  
            z_target = -y_cam      # 上
            
            return np.array([x_target, y_target, z_target])
            
        except Exception as e:
            logger.error(f"Error in 3D point calculation: {e}")
            return None
    
    def get_3d_pts_color_pixel(self, color_x, color_y):
        """
        从彩色图像像素获取3D坐标
        用于对齐深度图像的情况
        
        Args:
            color_x, color_y: 彩色图像像素坐标
            
        Returns:
            np.array: 3D坐标
        """
        # 对于对齐深度，彩色和深度图像坐标一一对应
        return self.get_3d_pts(color_x, color_y)
    
    
    def capture_target_coordinate(self, target_class, camera=None, timeout=60.0):
        """
        阻塞式获取目标坐标（按需检测版本）
        
        Args:
            camera: RealsenseCameraClientNode实例（可选）
            timeout: 超时时间（秒）
            
        Returns:
            np.array: 3D坐标 [x, y, z] 或 None（超时或失败）
        """
        logger.info(f"Starting target capture for '{target_class}'...")

        # 检查类别
        names = self.model.names
        cls_target = [k for k, v in names.items() if v.lower() == target_class.lower()]
        if not cls_target:
            raise ValueError(f"类别 '{target_class}' 不在模型中！")
        self.cls_target = cls_target[0]
        
        # 初始化相机
        if camera is None:
            logger.info("Initializing camera...")
            try:
                self.camera = RealsenseCameraClientNode()
                camera_initialized = True
            except Exception as e:
                logger.error(f"Failed to initialize camera: {e}")
                return None
        else:
            self.camera = camera
            camera_initialized = False
        
        # 创建显示窗口
        window_created = False
        try:
            cv2.namedWindow('Target Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Target Detection', 800, 600)
            window_created = True
            logger.info("Display window created")
        except Exception as e:
            logger.warning(f"Failed to create display window: {e}")
        
        # 清空历史点集
        self.collected_points.clear()
        start_time = time.time()
        frame_count = 0
        detection_count = 0
        
        try:
            while time.time() - start_time < timeout:
                frame_count += 1
                
                # 获取彩色图像
                color_image = self.camera.get_color_image()
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                if color_image is None:
                    logger.debug("Waiting for color image...")
                    time.sleep(0.033)
                    continue
                
                # 运行YOLO检测
                frame_start_time = time.time()
                results = self.model(color_image, verbose=False)[0]
                inference_time = (time.time() - frame_start_time) * 1000
                
                # 创建显示图像
                display_img = color_image.copy()
                boxes = results.boxes
                
                current_detection = None
                best_confidence = 0.0
                
                if boxes is not None and len(boxes) > 0:
                    for b in boxes:
                        # 检查是否为目标类别
                        if int(b.cls) == self.cls_target:
                            # 获取边界框和置信度
                            x1, y1, x2, y2 = map(int, b.xyxy[0].cpu().numpy())
                            conf = float(b.conf[0].cpu().numpy()) if b.conf is not None else 0.0
                            
                            # 选择置信度最高的检测
                            if conf > best_confidence and conf > 0.5:  # 置信度阈值
                                best_confidence = conf
                                
                                # 计算中心点（彩色图像坐标）
                                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                                
                                # 获取深度值
                                depth = self.camera.get_depth_value(cx, cy)
                                
                                if depth is not None and 0.1 < depth < 3.0:  # 有效深度范围
                                    # 获取3D坐标（使用彩色像素坐标，因为深度已对齐）
                                    pt_3d = self.get_3d_pts_color_pixel(cx, cy)
                                    
                                    if pt_3d is not None:
                                        current_detection = {
                                            'point_2d': (cx, cy),
                                            'point_3d': pt_3d,
                                            'bbox': (x1, y1, x2, y2),
                                            'confidence': conf,
                                            'depth': depth
                                        }
                
                # 处理检测结果
                if current_detection is not None:
                    detection_count += 1
                    
                    # 添加到历史点集
                    self.collected_points.append(current_detection['point_3d'])
                    
                    # 在图像上绘制结果
                    x1, y1, x2, y2 = current_detection['bbox']
                    cx, cy = current_detection['point_2d']
                    pt_3d = current_detection['point_3d']
                    conf = current_detection['confidence']
                    depth = current_detection['depth']
                    
                    # 绘制边界框和中心点
                    cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(display_img, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # 显示信息
                    info_lines = [
                        f"{self.target_class} ({conf:.2f})",
                        f"3D: ({pt_3d[0]:.2f}, {pt_3d[1]:.2f}, {pt_3d[2]:.2f})m",
                        f"Depth: {depth:.2f}m"
                    ]
                    
                    for i, line in enumerate(info_lines):
                        y_pos = y1 - 30 + i * 20
                        cv2.putText(display_img, line, 
                                   (x1, max(20, y_pos)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # 显示统计信息
                elapsed = time.time() - frame_start_time
                fps = 1.0 / elapsed if elapsed > 0 else 0
                
                status_lines = [
                    f"FPS: {fps:.1f} | Inference: {inference_time:.1f}ms",
                    f"Detections: {detection_count} | Stable: {len(self.collected_points)}/{self.confidence_frames}",
                    f"Press 'q' to quit"
                ]
                
                for i, line in enumerate(status_lines):
                    y_pos = 30 + i * 25
                    cv2.putText(display_img, line, 
                               (10, y_pos), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # 显示稳定性指示
                if len(self.collected_points) >= self.confidence_frames:
                    cv2.putText(display_img, "STABLE", 
                               (display_img.shape[1] - 100, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 显示图像（如果窗口创建成功）
                if window_created:
                    cv2.imshow('Target Detection', display_img)
                    key = cv2.waitKey(1) & 0xFF
                    
                    # 处理按键
                    if key == ord('q'):
                        logger.info("Quit requested by user")
                        break
                
                # 检查稳定检测
                if len(self.collected_points) >= self.confidence_frames:
                    # 计算平均位置和稳定性
                    recent_points = list(self.collected_points)[-self.confidence_frames:]
                    avg_point = np.mean(recent_points, axis=0)
                    std_dev = np.std(recent_points, axis=0)
                    stability = np.mean(std_dev)
                    
                    # 如果稳定，返回结果
                    if stability < 0.05:  # 5厘米稳定性阈值
                        logger.info(f"Stable detection! Position: {avg_point}, Stability: {stability:.3f}m")
                        return avg_point
                
                # 显示进度
                total_elapsed = time.time() - start_time
                if int(total_elapsed) % 5 == 0 and int(total_elapsed) > 0:
                    logger.info(f"  Elapsed: {total_elapsed:.1f}s, "
                               f"Collected: {len(self.collected_points)}/{self.confidence_frames}")
            
            logger.warning(f"Timeout after {timeout}s")
            
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Error in target capture: {str(e)}", exc_info=True)
        finally:
            # 清理资源
            if window_created:
                cv2.destroyWindow('Target Detection')
                logger.info("Display window destroyed")
            
            # 如果我们在内部初始化了相机，需要清理
            if camera_initialized and hasattr(self.camera, 'destroy_node'):
                try:
                    self.camera.destroy_node()
                    logger.info("Camera node destroyed")
                except:
                    pass
            
            # 重置状态
            self.collected_points.clear()
        
        return None
