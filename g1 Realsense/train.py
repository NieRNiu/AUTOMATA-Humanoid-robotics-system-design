from ultralytics import YOLO
import yaml
import os

# 创建数据集配置文件时使用绝对路径
dataset_config = {
    'path': os.path.abspath('./yolo_dataset'), 
    'train': 'train/images',
    'val': 'val/images',
    'names': {0: 'Ball', 1: 'Post', 2: "L", 3: "T", 4: "X"}
}

config_path = './yolov11_goalpost_ball.yaml'
with open(config_path, 'w') as f:
    yaml.dump(dataset_config, f)

# 初始化模型（选择不同规模的模型）
model = YOLO('yolo11s.pt')  # 官方预训练模型
# 可选模型：yolov11n/s/m/l/x.pt （从轻量到大型）

# 训练参数配置
train_params = {
    'data': config_path,
    'epochs': 600, 
    'batch': 16,
    'imgsz': 320 * 2,  
    'device': '0',  # GPU设备ID，CPU使用'cpu'
    'workers': 8,
    'optimizer': 'AdamW',  # 可选 SGD/Adam/AdamW等
    'lr0': 0.01,
    'lrf': 0.1,
    'cos_lr': True,
    'project': './runs/train',  # 训练结果保存目录
    'name': 'football_detection_v2',
    'exist_ok': True,  # 允许覆盖已有训练结果
    'augment': True,  # 启用数据增强
    'save_period': 10, # 每10个epoch保存一次模型
    'amp': True,       # 启用自动混合精度训练 
    'patience': 100
} 

# 开始训练
results = model.train(**train_params)

# 验证最佳模型
best_model = YOLO(os.path.join(train_params['project'], 
                train_params['name'], 'weights/best.pt'))
metrics = best_model.val(
    data = config_path,
    batch = 32,
    conf = 0.001,  # 验证置信度阈值
    iou = 0.6,      # NMS IoU阈值
    exist_ok = True
)



