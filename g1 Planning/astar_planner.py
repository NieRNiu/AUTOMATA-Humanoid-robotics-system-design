#!/usr/bin/env python3
"""
A* Path Planner for ROS1
Author: shenlanxueyuan
Email: 1347893477@qq.com
Organization: Shen Lan xue yuan
Date: 2025-03-28

This node implements an A* path planning algorithm with:
- Dynamic obstacle avoidance using inflated map
- Path smoothing and optimization
- Path visualization and saving
- Integration with ROS navigation stack
""" 

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import heapq
import math
from scipy import ndimage
import os
import time
import csv
import tf2_ros
import geometry_msgs.msg

class AStarPlanner:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('astar_planner', anonymous=True)
        
        # 参数
        self.map_data = None
        self.inflated_map = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.robot_radius = 0.5  # 机器人半径（米）
        self.inflation_radius_m = 0.5  # 膨胀半径（米） 
        
        # 起点和终点
        self.start_x = 0
        self.start_y = 0
        self.goal_x = 0
        self.goal_y = 0
        self.start_set = False  # 添加标志，表示是否设置了起点
        self.is_planning = False  # 添加标志，表示是否正在规划路径  
        
        # 路径
        self.path = None
        
        # 添加TF缓存区
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # ROS订阅和发布
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # 使用融合定位替代手动设置的初始位置
        self.odom_sub = rospy.Subscriber('/odometry/map', Odometry, self.odom_callback)
        
        # 保留现有的发布者
        self.path_pub = rospy.Publisher('/astar_path', Path, queue_size=10)
        self.inflated_map_pub = rospy.Publisher('/inflated_map', OccupancyGrid, queue_size=1)
        
    def map_callback(self, data):
        """接收地图数据"""
        self.map_data = np.array(data.data).reshape((data.info.height, data.info.width))
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution
        self.map_origin_x = data.info.origin.position.x
        self.map_origin_y = data.info.origin.position.y
        
        # 创建膨胀地图
        self.create_inflated_map(data)
    
    def odom_callback(self, data):
        """从融合定位获取机器人当前位置作为起点"""
        if self.map_data is None:
            rospy.logwarn("尚未接收到地图数据，无法设置起始位置")
            return
            
        try:
            # 获取当前位置
            curr_x = data.pose.pose.position.x
            curr_y = data.pose.pose.position.y
            
            # 检查frame_id和child_frame_id并记录
            source_frame = data.header.frame_id
            child_frame = data.child_frame_id
            rospy.loginfo_throttle(1, f"接收到里程计消息，坐标系: {source_frame}->{child_frame}")
            
            # 将世界坐标转换为地图坐标
            start_x, start_y = self.world_to_map(curr_x, curr_y)
            self.start_x, self.start_y = start_x, start_y
            self.start_set = True
            
            # 打印当前位置，调试用
            rospy.loginfo_throttle(1, f"当前机器人位置: 世界坐标({curr_x:.2f}, {curr_y:.2f}), 地图坐标({start_x}, {start_y})")
        except Exception as e:
            rospy.logerr(f"处理里程计数据时出错: {e}")
    
    def create_inflated_map(self, map_msg):
        """创建膨胀地图"""
        # 计算膨胀半径（像素）
        inflation_pixels = int(self.inflation_radius_m / self.map_resolution)
        
        try:
            # 创建二值障碍物地图
            binary_obstacle_map = np.zeros_like(self.map_data)
            binary_obstacle_map[self.map_data >= 50] = 1  # 障碍物为1，自由空间为0
            
            # 使用距离变换来膨胀障碍物
            from scipy import ndimage
            distance_from_obstacles = ndimage.distance_transform_edt(1 - binary_obstacle_map) * self.map_resolution
            
            # 创建膨胀地图
            self.inflated_map = np.copy(self.map_data)
            # 标记膨胀区域（距离障碍物小于膨胀半径的区域）
            inflation_mask = (distance_from_obstacles <= self.inflation_radius_m) & (self.map_data < 50)
            self.inflated_map[inflation_mask] = 99  # 将膨胀区域标记为99（接近障碍物）
            
            # 发布膨胀地图以便在RViz中可视化
            inflated_map_msg = OccupancyGrid()
            inflated_map_msg.header.stamp = rospy.Time.now()
            inflated_map_msg.header.frame_id = "map"
            
            # 完整复制地图元数据，确保所有必要字段都设置
            inflated_map_msg.info = map_msg.info
            
            # 确保地图数据正确格式化
            flat_data = self.inflated_map.flatten()
            inflated_map_msg.data = [int(cell) for cell in flat_data]
            
            # 发布膨胀地图
            self.inflated_map_pub.publish(inflated_map_msg)
            
            # 持续发布膨胀地图，确保RViz能够接收到
            rospy.Timer(rospy.Duration(1.0), self.publish_inflated_map_callback, oneshot=True)
            
        except Exception as e:
            rospy.logerr(f"创建膨胀地图时出错: {e}")
    
    def publish_inflated_map_callback(self, event):
        """定时回调，再次发布膨胀地图"""
        if self.inflated_map is not None and self.map_data is not None:
            try:
                inflated_map_msg = OccupancyGrid()
                inflated_map_msg.header.stamp = rospy.Time.now()
                inflated_map_msg.header.frame_id = "map"
                
                # 正确设置地图信息
                inflated_map_msg.info.resolution = self.map_resolution
                inflated_map_msg.info.width = self.map_width
                inflated_map_msg.info.height = self.map_height
                inflated_map_msg.info.origin.position.x = self.map_origin_x
                inflated_map_msg.info.origin.position.y = self.map_origin_y
                inflated_map_msg.info.origin.orientation.w = 1.0
                
                # 确保地图数据正确格式化和转换
                flat_data = self.inflated_map.flatten()
                inflated_map_msg.data = [int(cell) for cell in flat_data]
                
                # 发布消息
                self.inflated_map_pub.publish(inflated_map_msg)
                
                # 持续发布膨胀地图
                rospy.Timer(rospy.Duration(2.0), self.publish_inflated_map_callback, oneshot=True)
            except Exception as e:
                rospy.logerr(f"再次发布膨胀地图时出错: {e}")
        
    def goal_callback(self, data):
        """接收目标点"""
        if self.map_data is None:
            rospy.logwarn("尚未接收到地图数据，无法规划路径")
            return
            
        # 将世界坐标转换为地图坐标
        goal_x, goal_y = self.world_to_map(data.pose.position.x, data.pose.position.y)
        self.goal_x, self.goal_y = goal_x, goal_y
        
        # 如果起点已设置（通过odom_callback），则规划路径
        if self.start_set:
            self.is_planning = True 
            self.plan_path()
            # self.is_planning = False
        else:
            rospy.logwarn("尚未接收到有效的机器人位置，无法规划路径")
    
    def world_to_map(self, world_x, world_y):
        """将世界坐标转换为地图坐标"""
        x = int((world_x - self.map_origin_x) / self.map_resolution)
        y = int((world_y - self.map_origin_y) / self.map_resolution)
        return x, y
    
    def map_to_world(self, map_x, map_y):
        """将地图坐标转换为世界坐标"""
        world_x = map_x * self.map_resolution + self.map_origin_x
        world_y = map_y * self.map_resolution + self.map_origin_y
        return world_x, world_y
    
    def is_valid(self, x, y):
        """检查坐标是否有效，考虑膨胀地图"""
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return False
            
        # 使用膨胀地图进行检查，避开障碍物和膨胀区域
        if self.inflated_map is not None:
            return self.inflated_map[y][x] < 50  # 小于50视为可通行
        else:
            return self.map_data[y][x] < 50
    
    def get_neighbors(self, x, y):
        """获取邻居节点"""
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                # 对角线移动的代价为1.414
                cost = 1.414 if dx != 0 and dy != 0 else 1
                neighbors.append((nx, ny, cost))
        return neighbors
    
    def heuristic(self, x, y):
        """TODO::启发式函数：使用欧几里得距离"""
        return math.hypot(x - self.goal_x, y - self.goal_y)
    def plan_path(self):
        """使用A*算法规划路径"""
        if self.map_data is None:
            rospy.logwarn("尚未接收到地图数据，无法规划路径")
            return
        
        # 检查起点和终点是否有效
        if not self.is_valid(self.start_x, self.start_y):
            rospy.logwarn(f"起点({self.start_x}, {self.start_y})无效或处于障碍物中")
            return
            
        if not self.is_valid(self.goal_x, self.goal_y):
            rospy.logwarn(f"终点({self.goal_x}, {self.goal_y})无效或处于障碍物中")
            return
        
        # A*算法
        open_set = []
        closed_set = set()
        came_from = {}
        tmp_start_x = self.start_x
        tmp_start_y = self.start_y
        g_score = {(tmp_start_x, tmp_start_y): 0}
        f_score = {(tmp_start_x, tmp_start_y): self.heuristic(tmp_start_x, tmp_start_y)}
        
        heapq.heappush(open_set, (f_score[(tmp_start_x, tmp_start_y)], tmp_start_x, tmp_start_y))
        
        while open_set:
            _, current_x, current_y = heapq.heappop(open_set)
            current = (current_x, current_y)
            
            if current_x == self.goal_x and current_y == self.goal_y:
                # 构建路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((tmp_start_x, tmp_start_y))
                path.reverse()
                
                # 转换为世界坐标并发布
                self.publish_path(path)
                rospy.logwarn(f"起点({self.start_x}, {self.start_y})->终点({self.goal_x}, {self.goal_y})")
                rospy.logwarn(f"起点1({tmp_start_x}, {tmp_start_y})->终点({self.goal_x}, {self.goal_y})") 
                return
            
            closed_set.add(current)
            
            for neighbor_x, neighbor_y, cost in self.get_neighbors(current_x, current_y):
                # TODO:: 处理邻居节点
                node_n = (neighbor_x, neighbor_y)
                if node_n in closed_set:
                    continue
                
                g_n = g_score[current] + cost

                if node_n not in g_score or g_n < g_score[node_n]:
                    g_score[node_n] = g_n
                    f_score[node_n] = g_n + self.heuristic(neighbor_x, neighbor_y)
                    came_from[node_n] = current
                    heapq.heappush(open_set,(f_score[node_n], neighbor_x, neighbor_y))
        
        rospy.logwarn("无法找到路径")
    
    def publish_path(self, path):
        """发布路径消息"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        # 保存世界坐标路径点
        world_path = []
        
        for x, y in path:
            world_x, world_y = self.map_to_world(x, y)
            world_path.append({'x': world_x, 'y': world_y})
            
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0
            
            # 简单的朝向计算
            if len(path_msg.poses) > 0:
                prev_x = path_msg.poses[-1].pose.position.x
                prev_y = path_msg.poses[-1].pose.position.y
                yaw = math.atan2(world_y - prev_y, world_x - prev_x) 
                
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
            else:
                pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        # 如果路径为空，至少添加起点和终点
        if not path_msg.poses:
            start_world_x, start_world_y = self.map_to_world(self.start_x, self.start_y)
            goal_world_x, goal_world_y = self.map_to_world(self.goal_x, self.goal_y)
            
            world_path.append({'x': start_world_x, 'y': start_world_y})
            world_path.append({'x': goal_world_x, 'y': goal_world_y})
            
            start_pose = PoseStamped()
            start_pose.header.frame_id = "map"
            start_pose.pose.position.x = start_world_x
            start_pose.pose.position.y = start_world_y
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = goal_world_x
            goal_pose.pose.position.y = goal_world_y
            
            path_msg.poses.append(start_pose)
            path_msg.poses.append(goal_pose)
        
        # 保存路径到文件
        self.save_path_to_file(world_path)
        
        # 发布路径消息
        self.path_pub.publish(path_msg)
        rospy.loginfo(f"已发布包含{len(path_msg.poses)}个点的路径")
    
    def save_path_to_file(self, world_path):
        """将路径保存到CSV文件"""
        try:
            # 获取包的路径
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            
            # 创建path文件夹（如果不存在）
            path_dir = os.path.join(package_path, 'path')
            if not os.path.exists(path_dir):
                os.makedirs(path_dir)
            
            # 使用时间戳创建唯一文件名
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(path_dir, f'path_{timestamp}.csv')
            
            # 保存到CSV文件
            with open(filename, 'w', newline='') as csvfile:
                # 创建CSV写入器
                writer = csv.writer(csvfile)
                
                # 写入标题行
                writer.writerow(['x', 'y'])
                
                # 写入所有路径点
                for point in world_path:
                    writer.writerow([point['x'], point['y']])
            
            rospy.loginfo(f"路径已保存到CSV文件: {filename}")
        except Exception as e:
            rospy.logerr(f"保存路径到文件时出错: {e}")

if __name__ == '__main__':
    try:
        planner = AStarPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass