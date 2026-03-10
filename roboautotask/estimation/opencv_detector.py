import cv2
import numpy as np
import logging_mp
import time
import yaml
import os
from collections import deque

from roboautotask.configs.estimation import CONFIDENCE_FRAMES
from roboautotask.camera.realsense import RealsenseCameraClientNode

logger = logging_mp.get_logger(__name__)

# 默认HSV范围（全范围，检测任意颜色）
_DEFAULT_HSV = dict(h_min=0, h_max=180, s_min=50, s_max=255, v_min=50, v_max=255, invert=0)

_MIN_BLOB_AREA = 300  # 像素²，低于此面积的色块忽略
_PANEL_W = 480         # 显示面板宽度（px）
_PANEL_H = 360         # 显示面板高度（px）


def _get_motions_yaml_path(config_path: str | None = None) -> str:
    if config_path:
        return config_path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(os.path.dirname(current_dir)), "motions.yaml")


class ColorBlobDetector:
    """
    基于HSV颜色阈值的色块检测器。

    提供与 TargetDetection 相同的 capture_target_coordinate / get_3d_pts 接口，
    可直接替换用于 MotionExecutor。

    额外提供 tune_thresholds() 来打开可视化 Trackbar 调节界面并将结果保存回
    motions.yaml。
    """

    def __init__(
        self,
        confidence_frames: int = CONFIDENCE_FRAMES,
        config_path: str | None = None,
    ):
        self.confidence_frames = confidence_frames
        self._config_path = config_path  # motions.yaml 路径
        self.collected_points: deque = deque(maxlen=confidence_frames * 2)
        self.camera: RealsenseCameraClientNode | None = None
        self._motions_cfg: dict = self._load_yaml()

    # ------------------------------------------------------------------ #
    #  内部辅助
    # ------------------------------------------------------------------ #

    def _load_yaml(self) -> dict:
        yaml_path = _get_motions_yaml_path(self._config_path)
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            logger.error(f"Failed to load yaml '{yaml_path}': {e}")
            return {"items": {}}

    def _reload_yaml(self):
        self._motions_cfg = self._load_yaml()

    def _find_item(self, target_class: str) -> dict | None:
        """在 items 列表里按 name / label 查找对应条目。"""
        items = self._motions_cfg.get("items", {})
        for v in items.values():
            if not isinstance(v, dict):
                continue
            if v.get("name") == target_class or v.get("label") == target_class:
                return v
        return None

    def get_hsv_config(self, target_class: str) -> dict:
        """返回 target_class 对应的 HSV 配置，不存在则返回默认值。"""
        item = self._find_item(target_class)
        if item and "hsv_config" in item:
            cfg = {**_DEFAULT_HSV, **item["hsv_config"]}
            cfg["invert"] = int(item["hsv_config"].get("invert", 0))
            return cfg
        logger.warning(
            f"No hsv_config found for '{target_class}', using full-range defaults. "
            "Run `roboautotask-tune-color --item <name>` to configure."
        )
        return _DEFAULT_HSV.copy()

    def get_roi_config(self, target_class: str) -> list | None:
        """返回已配置的 ROI [x, y, w, h]（原图像素坐标），未配置则返回 None。"""
        item = self._find_item(target_class)
        if item and "roi" in item:
            return list(item["roi"])
        return None

    def save_item_config(self, target_class: str, hsv_cfg: dict, roi: list | None) -> bool:
        """同时将 hsv_config 和 roi 写回 motions.yaml。"""
        yaml_path = _get_motions_yaml_path(self._config_path)
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            saved = False
            for v in (data.get("items") or {}).values():
                if not isinstance(v, dict):
                    continue
                if v.get("name") == target_class or v.get("label") == target_class:
                    v["hsv_config"] = {k: int(val) for k, val in hsv_cfg.items()}
                    v["hsv_config"]["invert"] = int(hsv_cfg.get("invert", 0))
                    if roi is not None:
                        v["roi"] = [int(c) for c in roi]
                    elif "roi" in v:
                        del v["roi"]
                    saved = True
                    break
            if not saved:
                logger.error(f"Item '{target_class}' not found in yaml.")
                return False
            with open(yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
            logger.info(f"Config saved for '{target_class}': hsv={hsv_cfg}, roi={roi}")
            self._reload_yaml()
            return True
        except Exception as e:
            logger.error(f"Failed to save config: {e}")
            return False

    def save_hsv_config(self, target_class: str, cfg: dict) -> bool:
        """将 HSV 配置写回 motions.yaml，返回是否成功。"""
        yaml_path = _get_motions_yaml_path(self._config_path)
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            items = data.get("items", {})
            saved = False
            for v in items.values():
                if not isinstance(v, dict):
                    continue
                if v.get("name") == target_class or v.get("label") == target_class:
                    v["hsv_config"] = {k: int(val) for k, val in cfg.items()}
                    saved = True
                    break

            if not saved:
                logger.error(f"Item '{target_class}' not found in yaml; cannot save.")
                return False

            with open(yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False)

            logger.info(f"HSV config saved for '{target_class}': {cfg}")
            self._reload_yaml()
            return True
        except Exception as e:
            logger.error(f"Failed to save HSV config: {e}")
            return False

    # ------------------------------------------------------------------ #
    #  与 TargetDetection 相同的 3-D 接口
    # ------------------------------------------------------------------ #

    def get_3d_pts(self, x: int, y: int, depth_image=None) -> np.ndarray | None:
        """像素坐标 → 机器人坐标系 (+X=前, +Y=左, +Z=上)。"""
        try:
            point_3d = self.camera.pixel_to_3d(x, y, depth_image, use_depth_intrinsics=True)
            if point_3d is None:
                return None
            x_cam, y_cam, z_cam = point_3d
            return np.array([z_cam, -x_cam, -y_cam])
        except Exception as e:
            logger.error(f"Error getting 3D point: {e}")
            return None

    def get_3d_pts_color_pixel(self, color_x: int, color_y: int) -> np.ndarray | None:
        return self.get_3d_pts(color_x, color_y)

    # ------------------------------------------------------------------ #
    #  色块检测核心
    # ------------------------------------------------------------------ #

    def _build_mask(self, bgr_image: np.ndarray, cfg: dict) -> np.ndarray:
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lo = np.array([cfg["h_min"], cfg["s_min"], cfg["v_min"]])
        hi = np.array([cfg["h_max"], cfg["s_max"], cfg["v_max"]])
        mask = cv2.inRange(hsv, lo, hi)

        # H 通道跨 0 的情况（如红色，h_min > h_max）
        if cfg["h_min"] > cfg["h_max"]:
            lo2 = np.array([0, cfg["s_min"], cfg["v_min"]])
            hi2 = np.array([cfg["h_max"], cfg["s_max"], cfg["v_max"]])
            lo1 = np.array([cfg["h_min"], cfg["s_min"], cfg["v_min"]])
            hi1 = np.array([180, cfg["s_max"], cfg["v_max"]])
            mask = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1), cv2.inRange(hsv, lo2, hi2))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        if cfg.get("invert", 0):
            mask = cv2.bitwise_not(mask)
        return mask

    def _find_best_blob(self, mask: np.ndarray):
        """返回最大轮廓的 (cx, cy, contour) 或 (None, None, None)。"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < _MIN_BLOB_AREA:
            return None, None, None
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, None, None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy, largest

    # ------------------------------------------------------------------ #
    #  交互式预运行调节界面（纯 OpenCV，无 Qt Trackbar 依赖）
    # ------------------------------------------------------------------ #

    def tune_for_task(
        self,
        item_names: list[str],
        camera: RealsenseCameraClientNode | None = None,
        save: bool = True,
    ) -> bool:
        """
        每次运行前弹出的交互式调节界面（纯 OpenCV 实现，不依赖 Qt/GTK Trackbar）。

        布局（单窗口）
        ──────────────
        左面板 : 相机画面，叠加 ROI 框和检测轮廓。
        右面板 : 二值化 Mask。
        右侧边栏: 6 个手绘滑块（H/S/V Min/Max）+ 物品选项卡 + 按键说明。

        鼠标交互（左面板）
        ──────────────────
        左键拖动  →  绘制 / 更新 ROI 矩形。
        左键点击滑块轨道  →  直接跳值。
        左键拖动滑块滑块头  →  精细调节。

        按键
        ────
        Tab / 1-9   切换物品
        r           清除当前物品 ROI
        s           保存当前物品配置
        Enter       保存全部 & 继续运行（返回 True）
        q / Esc     放弃（返回 False）
        """
        if not item_names:
            return True

        if camera is not None:
            self.camera = camera
        elif self.camera is None:
            logger.info("Initializing camera for tuning...")
            self.camera = RealsenseCameraClientNode()

        # ── 工作状态 ───────────────────────────────────────────────────
        configs = {n: self.get_hsv_config(n) for n in item_names}
        rois    = {n: self.get_roi_config(n)  for n in item_names}
        cur_idx = [0]

        WIN = "Detection Tuner"
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

        # 布局常量
        CAM_W, CAM_H  = _PANEL_W, _PANEL_H           # 相机面板
        MASK_W        = _PANEL_W                       # mask 面板
        SIDE_W        = 260                            # 右侧控制栏宽度
        TAB_H         = 36                             # 物品标签高度
        LEGEND_H      = 50                             # 按键说明高度
        SL_LEFT       = SIDE_W // 6                   # 滑块左边距
        SL_RIGHT      = SIDE_W - 20                   # 滑块右边距
        SL_H          = 16                             # 滑块轨道高
        SL_THUMB      = 10                             # 滑块头半径
        TOTAL_W       = CAM_W + MASK_W + SIDE_W
        TOTAL_H       = max(CAM_H, _PANEL_H) + TAB_H + LEGEND_H

        cv2.resizeWindow(WIN, TOTAL_W, TOTAL_H)

        # 滑块定义：(label, key, max_val)
        SLIDERS = [
            ("H Min", "h_min", 180),
            ("H Max", "h_max", 180),
            ("S Min", "s_min", 255),
            ("S Max", "s_max", 255),
            ("V Min", "v_min", 255),
            ("V Max", "v_max", 255),
        ]
        # invert 是布尔量，不走滑块，用 i 键切换
        N_SL = len(SLIDERS)
        SL_ROW_H = (CAM_H - 20) // N_SL   # 每行滑块高度

        def _sl_y(i: int) -> int:
            """第 i 个滑块中心线 y 坐标（在侧边栏局部坐标系）"""
            return 24 + i * SL_ROW_H + SL_ROW_H // 2

        def _val_to_x(val: int, max_val: int) -> int:
            return SL_LEFT + int((val / max_val) * (SL_RIGHT - SL_LEFT))

        def _x_to_val(x: int, max_val: int) -> int:
            v = int((x - SL_LEFT) / (SL_RIGHT - SL_LEFT) * max_val)
            return max(0, min(max_val, v))

        # ── 鼠标状态 ────────────────────────────────────────────────────
        # ROI 绘制（在相机面板上）
        roi_draw   = {"active": False, "p0": None, "p1": None}
        scale_info = {"sx": 1.0, "sy": 1.0}
        # 滑块拖动（在右侧边栏上）
        sl_drag    = {"active": False, "idx": -1}

        def _mouse(event, mx, my, flags, param):
            cur_name = item_names[cur_idx[0]]
            cfg      = configs[cur_name]

            # ── 左面板：ROI 绘制 ─────────────────────────────────────
            if mx < CAM_W and my < CAM_H:
                if event == cv2.EVENT_LBUTTONDOWN:
                    roi_draw["active"] = True
                    roi_draw["p0"]     = (mx, my)
                    roi_draw["p1"]     = (mx, my)
                    sl_drag["active"]  = False
                elif event == cv2.EVENT_MOUSEMOVE and roi_draw["active"]:
                    roi_draw["p1"] = (mx, my)
                elif event == cv2.EVENT_LBUTTONUP and roi_draw["active"]:
                    roi_draw["active"] = False
                    roi_draw["p1"]     = (mx, my)
                    sx, sy = scale_info["sx"], scale_info["sy"]
                    x0d, y0d = roi_draw["p0"]
                    x1d, y1d = roi_draw["p1"]
                    xi = int(min(x0d, x1d) / sx)
                    yi = int(min(y0d, y1d) / sy)
                    wi = int(abs(x1d - x0d) / sx)
                    hi = int(abs(y1d - y0d) / sy)
                    if wi > 10 and hi > 10:
                        rois[cur_name] = [xi, yi, wi, hi]
                return

            # ── 右侧边栏：滑块交互 ──────────────────────────────────
            side_x  = mx - (CAM_W + MASK_W)   # 侧边栏局部 x
            side_y  = my                        # 侧边栏局部 y（与主画面同高）
            if side_x < 0 or side_x > SIDE_W:
                return
            if my >= CAM_H:
                return

            if event == cv2.EVENT_LBUTTONDOWN:
                # 找最近的滑块
                for i, (_, key, max_val) in enumerate(SLIDERS):
                    cy_sl = _sl_y(i)
                    if abs(side_y - cy_sl) <= SL_ROW_H // 2:
                        sl_drag["active"] = True
                        sl_drag["idx"]    = i
                        cfg[SLIDERS[i][1]] = _x_to_val(side_x, SLIDERS[i][2])
                        roi_draw["active"] = False
                        break
            elif event == cv2.EVENT_MOUSEMOVE and sl_drag["active"]:
                i = sl_drag["idx"]
                cfg[SLIDERS[i][1]] = _x_to_val(side_x, SLIDERS[i][2])
            elif event == cv2.EVENT_LBUTTONUP:
                sl_drag["active"] = False

        cv2.setMouseCallback(WIN, _mouse)

        def _switch(new_idx: int):
            cur_idx[0] = new_idx % len(item_names)
            roi_draw.update({"active": False, "p0": None, "p1": None})
            sl_drag["active"] = False

        def _draw_sliders(side: np.ndarray, cfg: dict):
            """在 side (H x SIDE_W BGR) 上绘制 6 个滑块。"""
            for i, (label, key, max_val) in enumerate(SLIDERS):
                cy_sl = _sl_y(i)
                # 轨道
                cv2.line(side,
                         (SL_LEFT, cy_sl), (SL_RIGHT, cy_sl),
                         (80, 80, 80), SL_H, cv2.LINE_AA)
                # 已选部分（亮蓝）
                thumb_x = _val_to_x(cfg[key], max_val)
                cv2.line(side,
                         (SL_LEFT, cy_sl), (thumb_x, cy_sl),
                         (180, 120, 40), SL_H, cv2.LINE_AA)
                # 滑块头
                cv2.circle(side, (thumb_x, cy_sl), SL_THUMB,
                           (0, 220, 255), -1, cv2.LINE_AA)
                cv2.circle(side, (thumb_x, cy_sl), SL_THUMB,
                           (0, 0, 0), 1, cv2.LINE_AA)
                # 标签 + 数值
                cv2.putText(side, f"{label}: {cfg[key]}",
                            (4, cy_sl - SL_H // 2 - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

        # ── 主循环 ──────────────────────────────────────────────────────
        result = False
        try:
            while True:
                cur_name = item_names[cur_idx[0]]
                cfg      = configs[cur_name]
                roi      = rois[cur_name]

                # 获取相机帧
                color_img = self.camera.get_color_image()
                if color_img is None:
                    time.sleep(0.033)
                    continue
                bgr = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
                orig_h, orig_w = bgr.shape[:2]

                sx = CAM_W / orig_w
                sy = CAM_H / orig_h
                scale_info["sx"] = sx
                scale_info["sy"] = sy

                # ROI 区域裁剪 + mask 计算
                if roi:
                    rx, ry, rw, rh = [max(0, int(v)) for v in roi]
                    rw = min(rw, orig_w - rx)
                    rh = min(rh, orig_h - ry)
                    if rw > 0 and rh > 0:
                        roi_bgr = bgr[ry:ry + rh, rx:rx + rw]
                        roi_off = (rx, ry)
                    else:
                        roi_bgr = bgr
                        roi_off = (0, 0)
                        roi     = None
                else:
                    roi_bgr = bgr
                    roi_off = (0, 0)

                mask_local = self._build_mask(roi_bgr, cfg)
                lx, ly, contour = self._find_best_blob(mask_local)
                fcx = (lx + roi_off[0]) if lx is not None else None
                fcy = (ly + roi_off[1]) if ly is not None else None

                # ── 左面板：相机 ─────────────────────────────────────
                left = cv2.resize(bgr, (CAM_W, CAM_H))

                if roi:
                    rx, ry, rw, rh = roi
                    rv1 = (int(rx * sx),            int(ry * sy))
                    rv2 = (int((rx + rw) * sx),     int((ry + rh) * sy))
                    cv2.rectangle(left, rv1, rv2, (0, 165, 255), 2)
                    cv2.putText(left, "ROI", (rv1[0] + 4, rv1[1] + 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

                if roi_draw["p0"] is not None and roi_draw["p1"] is not None:
                    cv2.rectangle(left, roi_draw["p0"], roi_draw["p1"],
                                  (0, 255, 255), 1)

                if contour is not None:
                    cnt = contour.astype(np.float32).copy()
                    cnt[:, 0, 0] = (cnt[:, 0, 0] + roi_off[0]) * sx
                    cnt[:, 0, 1] = (cnt[:, 0, 1] + roi_off[1]) * sy
                    cv2.drawContours(left, [cnt.astype(int)], -1, (0, 255, 0), 2)
                if fcx is not None:
                    dx, dy = int(fcx * sx), int(fcy * sy)
                    cv2.circle(left, (dx, dy), 8, (0, 255, 0), -1)
                    cv2.putText(left, f"({fcx},{fcy})",
                                (dx + 10, dy), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, (0, 255, 0), 1)

                cv2.putText(left, f"Camera | {cur_name}",
                            (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 0), 1)
                cv2.putText(left, "Drag to set ROI",
                            (6, CAM_H - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.40, (120, 120, 120), 1)

                # ── 右面板：mask ─────────────────────────────────────
                if roi:
                    full_mask = np.zeros((orig_h, orig_w), dtype=np.uint8)
                    rx, ry, rw, rh = [max(0, int(v)) for v in roi]
                    full_mask[ry:ry + rh, rx:rx + rw] = mask_local
                else:
                    full_mask = mask_local
                right = cv2.resize(full_mask, (MASK_W, _PANEL_H))
                right = cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)
                blob_ok  = fcx is not None
                blob_txt = f"DETECTED ({fcx},{fcy})" if blob_ok else "NOT DETECTED"
                blob_clr = (0, 255, 0) if blob_ok else (0, 0, 255)
                inv_lbl  = "INVERT: ON" if cfg.get("invert", 0) else "INVERT: off"
                inv_clr  = (0, 165, 255) if cfg.get("invert", 0) else (80, 80, 80)
                cv2.putText(right, "Mask",
                            (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 0), 1)
                cv2.putText(right, inv_lbl,
                            (6, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.48, inv_clr, 1)
                cv2.putText(right, blob_txt,
                            (6, _PANEL_H - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.50, blob_clr, 2)

                # ── 右侧边栏：滑块 ───────────────────────────────────
                side = np.zeros((CAM_H, SIDE_W, 3), dtype=np.uint8)
                _draw_sliders(side, cfg)

                # ── 标签栏 + 图例栏（底部）───────────────────────────
                tab_bar    = np.zeros((TAB_H, TOTAL_W, 3), dtype=np.uint8)
                legend_bar = np.zeros((LEGEND_H, TOTAL_W, 3), dtype=np.uint8)

                tab_w = TOTAL_W // max(len(item_names), 1)
                for i, name in enumerate(item_names):
                    active = (i == cur_idx[0])
                    bg     = (50, 50, 90) if active else (20, 20, 20)
                    tx     = i * tab_w
                    cv2.rectangle(tab_bar, (tx, 0), (tx + tab_w - 2, TAB_H), bg, -1)
                    clr   = (0, 220, 255) if active else (100, 100, 100)
                    short = (name[:18] + "…") if len(name) > 20 else name
                    cv2.putText(tab_bar, f"[{i+1}]{short}",
                                (tx + 4, TAB_H - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, clr, 1)

                cv2.putText(legend_bar,
                            "Tab/1-9=switch | r=clear ROI | i=toggle invert | s=save | "
                            "Enter=save all & run | q=abort",
                            (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                            (155, 155, 155), 1)
                inv_flag = "ON" if cfg.get("invert", 0) else "off"
                cv2.putText(legend_bar,
                            f"H[{cfg['h_min']},{cfg['h_max']}]  "
                            f"S[{cfg['s_min']},{cfg['s_max']}]  "
                            f"V[{cfg['v_min']},{cfg['v_max']}]  "
                            f"Invert:{inv_flag}",
                            (6, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            (160, 160, 0), 1)

                # ── 合成 & 显示 ──────────────────────────────────────
                body    = np.hstack([left, right, side])
                display = np.vstack([body, tab_bar, legend_bar])
                cv2.imshow(WIN, display)

                # ── 按键 ─────────────────────────────────────────────
                key = cv2.waitKey(30) & 0xFF

                if key in (ord("q"), 27):
                    result = False
                    break
                elif key == 13:                     # Enter
                    if save:
                        for name in item_names:
                            self.save_item_config(name, configs[name], rois[name])
                    result = True
                    break
                elif key == ord("s"):
                    if save:
                        self.save_item_config(cur_name, cfg, roi)
                        logger.info(f"Saved config for '{cur_name}'")
                elif key == ord("i"):               # toggle invert
                    cfg["invert"] = 0 if cfg.get("invert", 0) else 1
                    logger.info(f"Invert {'ON' if cfg['invert'] else 'off'} for '{cur_name}'")
                elif key == ord("r"):
                    rois[cur_name] = None
                    roi_draw.update({"active": False, "p0": None, "p1": None})
                    logger.info(f"ROI cleared for '{cur_name}'")
                elif key == 9:                      # Tab
                    _switch(cur_idx[0] + 1)
                elif ord("1") <= key <= ord("9"):
                    idx = key - ord("1")
                    if idx < len(item_names):
                        _switch(idx)

        finally:
            try:
                cv2.destroyWindow(WIN)
            except Exception:
                pass

        return result

    def tune_thresholds(
        self,
        target_class: str,
        camera: RealsenseCameraClientNode | None = None,
        save: bool = True,
    ) -> dict:
        """向后兼容的单物品调节接口，内部调用 tune_for_task。返回最终 HSV 配置字典。"""
        self.tune_for_task([target_class], camera=camera, save=save)
        return self.get_hsv_config(target_class)

    # ------------------------------------------------------------------ #
    #  主检测接口（与 TargetDetection 相同签名）
    # ------------------------------------------------------------------ #

    def capture_target_coordinate(
        self,
        target_class: str,
        camera: RealsenseCameraClientNode | None = None,
        timeout: float = 60.0,
    ) -> np.ndarray | None:
        """
        阻塞式检测目标色块并返回 3-D 坐标。
        若该物品在 motions.yaml 中配置了 roi，则仅在 ROI 区域内检测。

        Returns:
            np.array([x, y, z]) — (+X=前, +Y=左, +Z=上)，超时返回 None
        """
        logger.info(f"[OpenCV] Detecting '{target_class}'...")

        if camera is not None:
            self.camera = camera
        elif self.camera is None:
            logger.info("Initializing camera...")
            self.camera = RealsenseCameraClientNode()

        cfg = self.get_hsv_config(target_class)
        roi = self.get_roi_config(target_class)
        logger.info(f"HSV config: {cfg}  ROI: {roi}")

        win = "Color Detection (OpenCV)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 800, 480)

        self.collected_points.clear()
        start_time = time.time()

        try:
            while time.time() - start_time < timeout:
                color_image = self.camera.get_color_image()
                if color_image is None:
                    time.sleep(0.033)
                    continue
                bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                orig_h, orig_w = bgr.shape[:2]

                # 应用 ROI
                if roi:
                    rx, ry, rw, rh = [max(0, int(v)) for v in roi]
                    rw = min(rw, orig_w - rx)
                    rh = min(rh, orig_h - ry)
                    if rw > 0 and rh > 0:
                        roi_bgr = bgr[ry:ry + rh, rx:rx + rw]
                        roi_off = (rx, ry)
                    else:
                        roi_bgr = bgr
                        roi_off = (0, 0)
                else:
                    roi_bgr = bgr
                    roi_off = (0, 0)

                mask = self._build_mask(roi_bgr, cfg)
                cx, cy, contour = self._find_best_blob(mask)

                # 换算回全图坐标
                fcx = (cx + roi_off[0]) if cx is not None else None
                fcy = (cy + roi_off[1]) if cy is not None else None

                display = bgr.copy()
                if roi:
                    rx, ry, rw, rh = [max(0, int(v)) for v in roi]
                    cv2.rectangle(display, (rx, ry), (rx + rw, ry + rh),
                                  (0, 165, 255), 2)
                if contour is not None:
                    cnt = contour.copy()
                    cnt[:, 0, 0] += roi_off[0]
                    cnt[:, 0, 1] += roi_off[1]
                    cv2.drawContours(display, [cnt], -1, (0, 255, 0), 2)

                if fcx is not None:
                    depth = self.camera.get_depth_value(fcx, fcy)
                    if depth is not None and 0.1 < depth < 3.0:
                        pt_3d = self.get_3d_pts_color_pixel(fcx, fcy)
                        if pt_3d is not None:
                            self.collected_points.append(pt_3d)
                            cv2.circle(display, (fcx, fcy), 8, (0, 255, 0), -1)
                            cv2.putText(
                                display,
                                f"3D: ({pt_3d[0]:.3f}, {pt_3d[1]:.3f}, {pt_3d[2]:.3f})",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                            )

                n       = len(self.collected_points)
                elapsed = time.time() - start_time
                cv2.putText(
                    display, f"Collected: {n}/{self.confidence_frames}  [{elapsed:.1f}s/{timeout:.0f}s]",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2,
                )
                cv2.putText(
                    display, f"Target: {target_class}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2,
                )
                cv2.imshow(win, display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    logger.info("Detection aborted by user.")
                    break

                if n >= self.confidence_frames:
                    pts = np.array(list(self.collected_points)[-self.confidence_frames:])
                    std = np.std(pts, axis=0)
                    if np.all(std < 0.05):  # 5 cm 稳定性阈值
                        result = np.mean(pts, axis=0)
                        logger.info(f"[OpenCV] Stable detection: {result}, std={std}")
                        return result
        finally:
            cv2.destroyWindow(win)

        logger.warning(f"[OpenCV] Detection timeout for '{target_class}'")
        return None
