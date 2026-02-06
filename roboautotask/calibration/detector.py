import cv2
import logging_mp
import numpy as np


logger = logging_mp.get_logger(__name__)


class ArucoDetector():
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.latest_view = None  # 最新可视化图像

        self.latest_markers = {}  # {"id": [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]}
        self.latest_centers = {}  # {"id": (x, y, z)}


    def detect(self, color_image: np.ndarray):
        """检测marker"""
        try:
            if color_image is None:
                return None

            display = color_image.copy()

            corners, ids, _ = self.detector.detectMarkers(color_image)
            markers = {}

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                for i, corner in enumerate(corners):
                    markers[int(ids[i][0])] = corner[0]

            self.latest_markers = markers.copy()
            self.latest_view = display
        except Exception as e:
            logger.error(f'Error in update_aruco_mark: {e}')
            return None
        
    def update_view_image(self):
        display = self.latest_view
        for id, center in self.latest_centers.items():
            # 在图片显示mark点xyz
            cx = int(np.mean(self.latest_markers[id][:, 0]))
            cy = int(np.mean(self.latest_markers[id][:, 1]))
            x, y, z = center
            txt = f'Mark ID {id}: {x:.2f}, {y:.2f}, {z:.2f}'
            (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(display, (cx-5, cy-25), (cx+w+5, cy), (0, 0, 0), -1)
            cv2.putText(display, txt, (cx, cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255, 255), 2)
        
        # 状态条
        status = f"Press 'r' to record | 'q' to quit | Detected: {len(self.latest_markers)} markers"
        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        self.latest_view = display

    def get_view_image(self):
        """获取显示图像"""
        img = self.latest_view.copy() if self.latest_view is not None else \
                    np.zeros((480, 640, 3), np.uint8)
        return img

    def clean(self):
        self.latest_markers = {}
        self.latest_centers = {}