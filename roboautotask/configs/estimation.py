# ================= 视觉与模型参数 =================
YOLO_MODEL_PATH = "models/yoloe-26x-seg.pt"
TARGET_CLASS = "cup" # 由于转为使用yaml，这个参数只是个摆设，已被弃用
CONFIDENCE_FRAMES = 7  # 连续检测多少帧后确认为有效目标
