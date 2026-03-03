import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading
import time
from collections import deque
import logging_mp


logger = logging_mp.get_logger(__name__)


class RealsenseCameraClientNode(Node):
    """
    ROS2相机客户端，从已启动的相机节点订阅图像和内参
    修正版：正确使用深度相机内参进行3D反投影
    """
    
    def __init__(self, 
                 color_topic='/camera/color/image_raw',
                 depth_topic='/camera/depth/image_rect_raw',
                 color_info_topic='/camera/color/camera_info',
                 depth_info_topic='/camera/depth/camera_info',
                 use_sim_time=False):
        """
        初始化相机客户端
        
        Args:
            color_topic: 彩色图像话题
            depth_topic: 深度图像话题
            color_info_topic: 彩色相机内参话题
            depth_info_topic: 深度相机内参话题
            use_sim_time: 是否使用仿真时间
        """
        super().__init__('roboautotask_realsense_camera_client_node')
        
        # CV桥接器
        self.bridge = CvBridge()
        
        # 数据缓存
        self.color_image = None
        self.depth_image = None
        self.color_info = None
        self.depth_info = None
        self.color_intrinsics = None
        self.depth_intrinsics = None
        
        # 深度尺度（对于16UC1格式）
        self.depth_scale = 0.001  # 毫米转米（16UC1）
        
        # 是否对齐深度到彩色
        self.is_aligned_depth = 'aligned' in depth_topic
        
        # 线程锁
        self.lock = threading.RLock()
        
        # 消息队列
        self.color_queue = deque(maxlen=5)
        self.depth_queue = deque(maxlen=5)
        
        # 话题名称
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.color_info_topic = color_info_topic
        self.depth_info_topic = depth_info_topic
        
        # 订阅话题
        self.create_subscriptions()
        
        # 等待第一帧数据
        self.wait_for_first_frame()
        
        logger.info("Camera client initialized")
        logger.info(f"Using aligned depth: {self.is_aligned_depth}")
    
    def create_subscriptions(self):
        """创建话题订阅"""
        # 彩色图像订阅
        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            10
        )
        
        # 深度图像订阅
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        # 彩色相机内参订阅
        self.color_info_sub = self.create_subscription(
            CameraInfo,
            self.color_info_topic,
            self.color_info_callback,
            10
        )
        
        # 深度相机内参订阅
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.depth_info_callback,
            10
        )
    
    def extract_intrinsics_from_msg(self, msg):
        """从CameraInfo消息提取内参"""
        return {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5],
            'width': msg.width,
            'height': msg.height,
            'distortion_model': msg.distortion_model,
            'distortion_coeffs': msg.d,
            'frame_id': msg.header.frame_id
        }
    
    def color_callback(self, msg):
        """彩色图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            
            with self.lock:
                self.color_image = cv_image
                self.color_queue.append(cv_image)
                
        except CvBridgeError as e:
            logger.error(f"CV Bridge error in color callback: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            # 检查深度图像编码格式
            if msg.encoding == '16UC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            elif msg.encoding == '32FC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                self.depth_scale = 1.0  # 32FC1已经是米制单位
            else:
                logger.warning(f"Unsupported depth encoding: {msg.encoding}")
                return
            
            with self.lock:
                self.depth_image = cv_image
                self.depth_queue.append(cv_image)
                
        except CvBridgeError as e:
            logger.error(f"CV Bridge error in depth callback: {e}")
    
    def color_info_callback(self, msg):
        """彩色相机内参回调"""
        try:
            with self.lock:
                self.color_info = msg
                self.color_intrinsics = self.extract_intrinsics_from_msg(msg)
                
                logger.debug(f"Color camera intrinsics received: "
                                      f"fx={self.color_intrinsics['fx']:.2f}, "
                                      f"fy={self.color_intrinsics['fy']:.2f}")
                
        except Exception as e:
            logger.error(f"Error in color info callback: {e}")
    
    def depth_info_callback(self, msg):
        """深度相机内参回调"""
        try:
            with self.lock:
                self.depth_info = msg
                self.depth_intrinsics = self.extract_intrinsics_from_msg(msg)
                
                logger.debug(f"Depth camera intrinsics received: "
                                      f"fx={self.depth_intrinsics['fx']:.2f}, "
                                      f"fy={self.depth_intrinsics['fy']:.2f}")
                
        except Exception as e:
            logger.error(f"Error in depth info callback: {e}")
    
    def wait_for_first_frame(self, timeout=10.0):
        """等待第一帧数据"""
        logger.info("Waiting for camera data...")
        
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout:
            with self.lock:
                if (self.color_image is not None and 
                    self.depth_image is not None and 
                    self.depth_intrinsics is not None):
                    logger.info("Camera data received!")
                    return True
            
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        logger.warning("Timeout waiting for camera data")
        return False
    
    def get_color_image(self):
        """获取彩色图像"""
        with self.lock:
            return self.color_image.copy() if self.color_image is not None else None
    
    def get_depth_image(self):
        """获取深度图像"""
        with self.lock:
            return self.depth_image.copy() if self.depth_image is not None else None
    
    def get_depth_value(self, x, y, depth_image=None):
        """
        获取指定像素的深度值（米）
        
        Args:
            x, y: 像素坐标（必须是深度图像坐标系下的坐标）
            depth_image: 深度图像（可选）
            
        Returns:
            float: 深度值（米）
        """
        with self.lock:
            if depth_image is None:
                depth_image = self.depth_image
            
            if depth_image is None:
                return None
            
            # 检查坐标范围
            height, width = depth_image.shape[:2]
            if not (0 <= x < width and 0 <= y < height):
                return None
            
            # 获取深度值
            depth_raw = depth_image[y, x]
            
            # 根据编码转换
            if depth_image.dtype == np.uint16:  # 16UC1
                if depth_raw == 0:  # 0表示无效深度
                    return None
                depth_meters = depth_raw * self.depth_scale
            elif depth_image.dtype == np.float32:  # 32FC1
                if depth_raw <= 0 or np.isnan(depth_raw) or np.isinf(depth_raw):
                    return None
                depth_meters = depth_raw
            else:
                return None
            
            return depth_meters
    
    def pixel_to_3d(self, x, y, depth=None, use_depth_intrinsics=True):
        """
        将像素坐标转换为3D坐标
        
        Args:
            x, y: 像素坐标
            depth: 深度值（米），如果为None则从深度图像获取
            use_depth_intrinsics: 是否使用深度相机内参（True=正确做法）
            
        Returns:
            np.array: 3D坐标 [x, y, z]（相机坐标系）
                     深度相机坐标系: +X=右, +Y=下, +Z=前
        """
        # 确定使用哪个内参
        if use_depth_intrinsics:
            intrinsics = self.depth_intrinsics
            if intrinsics is None:
                logger.error("Depth camera intrinsics not available")
                return None
        else:
            intrinsics = self.color_intrinsics
            if intrinsics is None:
                logger.error("Color camera intrinsics not available")
                return None
        
        # 获取深度值
        if depth is None:
            depth = self.get_depth_value(x, y)
            if depth is None:
                return None
        
        # 使用内参进行反投影
        fx = intrinsics['fx']
        fy = intrinsics['fy']
        cx = intrinsics['cx']
        cy = intrinsics['cy']
        
        # 归一化坐标
        x_normalized = (x - cx) / fx
        y_normalized = (y - cy) / fy
        
        # 3D坐标（深度相机坐标系）
        x_3d = x_normalized * depth
        y_3d = y_normalized * depth
        z_3d = depth
        
        return np.array([x_3d, y_3d, z_3d])
    
    def color_pixel_to_3d(self, color_x, color_y):
        """
        彩色图像像素转3D坐标（用于对齐深度图像）
        
        Args:
            color_x, color_y: 彩色图像像素坐标
            
        Returns:
            np.array: 3D坐标 [x, y, z]
        """
        if not self.is_aligned_depth:
            logger.warning("Using non-aligned depth image. Results may be inaccurate.")
        
        # 对于对齐深度，彩色和深度图像坐标一一对应
        return self.pixel_to_3d(color_x, color_y, use_depth_intrinsics=True)
    
    def is_data_available(self):
        """检查数据是否可用"""
        with self.lock:
            return (self.color_image is not None and 
                   self.depth_image is not None and 
                   self.depth_intrinsics is not None)
