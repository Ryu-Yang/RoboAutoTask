# ================= 视觉与模型参数 =================
YOLO_MODEL_PATH = "models/yoloe-26x-seg.pt"
TARGET_CLASS = "cup" # 由于转为使用yaml，这个参数只是个摆设，已被弃用
CONFIDENCE_FRAMES = 7  # 连续检测多少帧后确认为有效目标

# 检测方法默认值（可在 motions.yaml 的 detection_method 字段覆盖）
# "yolo"   — 使用 ultralytics YOLO 开放词汇检测（默认）
# "opencv" — 使用 HSV 色块检测（需为每个 item 配置 hsv_config）
DETECTION_METHOD = "opencv"
