from ultralytics import YOLO

model = YOLO("./runs/train/football_detection/weights/best.pt")

# 预测并保存结果
results = model.predict(
    source='./predict_data/vedio',
    save=True,          # 保存带标注的预测图片
    save_txt=True,       # 保存检测框的 txt 文件（YOLO 格式）
    save_conf=True,      # 保存置信度到 txt 文件
    save_crop=False,     # 保存裁剪的检测目标（可选）
    project='./results', # 指定保存路径（默认是 runs/detect）
    exist_ok=True        # 允许覆盖已有结果
)
