#!/usr/bin/env python3
"""
Point Cloud to PGM Map Converter
Author: shenlanxueyuan
Email: 1347893477@qq.com
Organization: Shen Lan xue yuan
Date: 2025-03-28
 
This script converts point cloud data (.pcd) to PGM map format (.pgm) for ROS navigation.
Features:
- Filters ground points based on Z-axis threshold
- Converts 3D point cloud to 2D occupancy grid
- Generates corresponding YAML configuration file
"""

import open3d as o3d
import numpy as np
from PIL import Image
import yaml

# 配置参数
RESOLUTION = 0.05  # 米/像素
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
Z_THRESH = 0.2  # Z轴过滤阈值（单位：米）

def main():
    # 读取点云数据
    pcd = o3d.io.read_point_cloud("map.pcd")
    points = np.asarray(pcd.points)
    
    # 过滤无效点
    valid_points = points[~np.isnan(points).any(axis=1)]
    if len(valid_points) == 0:
        raise ValueError("点云数据中没有有效点")
    
    # 新增：过滤地面点（Z轴最低部分）
    valid_points = valid_points[valid_points[:, 2] > Z_THRESH]  # Z坐标大于阈值的点保留
    if len(valid_points) == 0:
        raise ValueError("过滤地面点后没有剩余有效点")
    
    # 计算边界
    min_x, min_y = np.min(valid_points[:, 0]), np.min(valid_points[:, 1])
    max_x, max_y = np.max(valid_points[:, 0]), np.max(valid_points[:, 1])
    
    # 计算地图尺寸
    width = int(np.ceil((max_x - min_x) / RESOLUTION)) + 1
    height = int(np.ceil((max_y - min_y) / RESOLUTION)) + 1
    
    # 创建栅格地图
    grid_map = np.full((height, width), 255, dtype=np.uint8)
    
    # 填充障碍物（已过滤地面点）
    for x, y in valid_points[:, 0:2]:
        j = int((x - min_x) / RESOLUTION)
        i = int((max_y - y) / RESOLUTION)
        if 0 <= j < width and 0 <= i < height:
            grid_map[i, j] = 0  # 障碍物设为黑色
    
    # 保存PGM文件
    Image.fromarray(grid_map).save('map.pgm')
    
    # 计算原点坐标（左下角）
    origin_x = float(min_x)
    origin_y = float(max_y) - (height - 1) * RESOLUTION
    
    # TODO::生成YAML文件
    yaml_data = {
        'image': 'map.pgm',
        'resolution': RESOLUTION,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': OCCUPIED_THRESH,
        'free_thresh': FREE_THRESH
    }
    
    with open('map.yaml', 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=None)

if __name__ == "__main__":
    main()
