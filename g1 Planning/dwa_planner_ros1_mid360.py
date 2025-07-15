#!/usr/bin/env python3
"""
DWA (Dynamic Window Approach) Planner for ROS1
Author: shenlanxueyuan
Email: 1347893477@qq.com
Organization: Shen Lan xue yuan 
Date: 2025-03-28

This node implements a DWA-based local planner for robot navigation with:
- Dynamic obstacle avoidance
- Path following with A* path
- Smooth motion control
- Point cloud processing for obstacle detection
- Visualization tools for debugging
"""
 
import rospy
import math
import numpy as np
import random
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs

class DWAPlanner:
    def __init__(self):
        # 初始化节点
        rospy.init_node('dwa_planner', anonymous=True)
        
        # 状态变量
        self.goal_x = None
        self.goal_y = None
        self.odom_data = None
        self.astar_path = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.has_path = False
        self.has_goal = False
        self.goal_pose = None
        self.goal_reached = False
        self.waypoint_distance = 0.1  # 路点到达阈值
        
        # 避障状态
        self.obstacle_detected = False
        self.obstacle_avoidance_active = False
        self.last_obstacle = None  # 记录最后一个检测到的障碍物位置
        
        # 点云数据
        self.pointcloud_data = None
        self.obstacles = []  # 检测到的障碍物列表
        
        # 参数设置
        self.max_speed = 0.3  # 提高最大速度
        self.min_speed = 0.15  # 提高最小速度，确保避障时不会停止 0.12
        self.max_turn = 0.3  # 增加最大转向速度以便更灵活避障0.5——>0.3——>0.7——>0.8——>0.3
        self.preferred_turn_direction = None  # 记录优先转向方向
        self.step_time = 0.1
        self.safety_margin = 0.3
        self.detection_range = 1.2  # 障碍物检测范围
        self.min_obstacle_radius = 0.2  # 障碍物最小半径
        self.max_obstacle_radius = 1.0  # 障碍物最大半径
        self.min_obstacle_points = 100  # 判定为障碍物的最小点数
        self.clustering_distance = 0.3  # 聚类距离
        self.robot_radius = 0.3  # 机器人半径
        
        # 添加调试标志
        self.debug = True
        self.last_debug_time = rospy.Time.now()
        self.debug_interval = rospy.Duration(1) # 每秒打印一次调试信息 
        
        # 增加避障控制的平滑性
        self.last_cmd_vel = {'linear': 0.0, 'angular': 0.0}
        self.smooth_factor = 1.2  # 速度平滑因子0.3
        
        # TF缓存
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅者
        self.odom_sub = rospy.Subscriber('/odometry/map', Odometry, self.odom_callback, queue_size=1)
        self.path_sub = rospy.Subscriber('/astar_path', Path, self.path_callback, queue_size=1)
        self.pointcloud_sub = rospy.Subscriber('/aligned', PointCloud2, self.pointcloud_callback, queue_size=1)
        
        # 发布者
        self.cmd_publisher = rospy.Publisher('/dwa_cmd_vel', Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('/dwa_paths', Marker, queue_size=1)
        self.obstacle_status_pub = rospy.Publisher('/obstacle_status', Point, queue_size=1)
        self.obstacles_marker_pub = rospy.Publisher('/detected_obstacles', MarkerArray, queue_size=1)
        self.filtered_points_pub = rospy.Publisher('/filtered_points', Marker, queue_size=1)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(self.step_time), self.movement_loop)
        
        rospy.loginfo("DWA planner started, waiting for data...")

    def odom_callback(self, msg):
        self.odom_data = msg
        
    def pointcloud_callback(self, msg):
        """处理点云数据"""
        self.pointcloud_data = msg
        
    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("收到空路径，忽略")
            return
        
        # 清空当前路径点
        self.waypoints.clear()
        
        # 提取路径中的路点
        for pose in msg.poses:
            self.waypoints.append(pose)
        
        self.has_path = True
        self.current_waypoint_index = 0
        self.goal_reached = False  # 重置目标到达标志
        
        rospy.loginfo(f"收到新路径，包含 {len(self.waypoints)} 个路点，frame_id={msg.header.frame_id}")
        
        # 设置最终目标
        if self.waypoints:
            self.goal_pose = self.waypoints[-1]
            self.has_goal = True
            # 更新全局目标坐标
            self.goal_x = self.goal_pose.pose.position.x
            self.goal_y = self.goal_pose.pose.position.y
            rospy.loginfo(f"设置新目标: x={self.goal_x}, y={self.goal_y}")
            
            # 确保从避障模式重置
            if self.obstacle_avoidance_active:
                self.obstacle_avoidance_active = False
                self.preferred_turn_direction = None
                self.publish_obstacle_status(0)
                rospy.loginfo("收到新路径，重置避障状态")

    def process_pointcloud(self):
        """处理点云数据，识别障碍物"""
        if self.pointcloud_data is None or self.odom_data is None:
            return []
            
        # 获取机器人当前位置和朝向
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # 确保点云和机器人在同一坐标系
        pc_frame_id = self.pointcloud_data.header.frame_id
        odom_frame_id = self.odom_data.header.frame_id
        
        # 提取点云数据
        try:
            # 检查坐标系是否需要转换
            transform_points = False
            transform = None
            
            if pc_frame_id != odom_frame_id:
                rospy.loginfo(f"坐标系需要转换: {pc_frame_id} -> {odom_frame_id}")

                try:
                    # 获取从点云坐标系到里程计坐标系的变换
                    transform = self.tf_buffer.lookup_transform(
                        odom_frame_id,
                        pc_frame_id,
                        rospy.Time(0),
                        rospy.Duration(1.0)
                    )
                    transform_points = True
                    rospy.loginfo_throttle(5.0, f"应用坐标转换: {pc_frame_id} -> {odom_frame_id}")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn_throttle(5.0, f"无法获取坐标转换: {e}")
                    # 继续处理，但不进行转换
            
            # 提取点云中的点
            points = list(pc2.read_points(self.pointcloud_data, field_names=("x", "y", "z"), skip_nans=True))
            
            # 过滤点：只保留前方扇区的点，以及水平高度接近的点
            filtered_points = []
            for point in points:
                # 如果需要转换坐标系
                if transform_points and transform:
                    rospy.loginfo(f"需要转换坐标系: {pc_frame_id} -> {odom_frame_id}")
                    # 创建点云中的点的PoseStamped消息
                    point_pose = PoseStamped()
                    point_pose.header.frame_id = pc_frame_id
                    point_pose.pose.position.x = point[0]
                    point_pose.pose.position.y = point[1]
                    point_pose.pose.position.z = point[2]
                    point_pose.pose.orientation.w = 1.0
                    
                    # 转换点到正确的坐标系
                    try:
                        transformed_point = tf2_geometry_msgs.do_transform_pose(point_pose, transform)
                        # 更新点坐标
                        point = (
                            transformed_point.pose.position.x,
                            transformed_point.pose.position.y,
                            transformed_point.pose.position.z
                        )
                    except Exception as e:
                        rospy.logwarn_throttle(5.0, f"点转换失败: {e}")
                        continue
                
                # rospy.loginfo(f"过滤前的点云数量: {len(points)}")
                
                # 计算点相对于机器人的距离和角度
                dx = point[0] - robot_x
                dy = point[1] - robot_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # 只保留在检测范围内的点
                if distance > self.detection_range:
                    continue
                
                # 计算点相对于机器人的角度
                angle = math.atan2(dy, dx)
                relative_angle = self.normalize_angle(angle - yaw)
                
                # 只保留前方±90度范围内的点
                if abs(relative_angle) > math.pi/2:
                    continue
                    
                # 过滤高度(假设z为高度)
                if point[2] < 0:  # 保留z值在0.1米范围外的点
                    continue

                # print(point[2])
                
                # 添加到过滤后的点集
                filtered_points.append((point[0], point[1], distance))
            
            
            # 打印过滤后的点云数量
            rospy.loginfo(f"过滤后的点云数量: {len(filtered_points)}")
            
            # 使用简单聚类算法识别障碍物
            obstacles = self.cluster_points(filtered_points)
            
            # 更新障碍物列表
            self.obstacles = obstacles
            
            # 发布障碍物可视化
            self.publish_obstacles_visualization()
            
            # 发布过滤后的点云可视化
            self.publish_filtered_points(filtered_points)
            
            return obstacles
            
        except Exception as e:
            rospy.logerr(f"处理点云数据时出错: {e}")
            return []
    
    def cluster_points(self, filtered_points):
        """简单的聚类算法将点聚合为障碍物"""
        if not filtered_points:
            return []
            
        # 未分配的点
        unassigned_points = filtered_points.copy()
        clusters = []
        
        while unassigned_points:
            # 创建新簇
            current_cluster = []
            seed_point = unassigned_points.pop(0)
            current_cluster.append(seed_point)
            
            # 查找簇中所有相邻点
            i = 0
            while i < len(current_cluster):
                current_point = current_cluster[i]
                i += 1
                
                # 寻找当前点的邻居
                j = 0
                while j < len(unassigned_points):
                    neighbor = unassigned_points[j]
                    
                    # 计算两点距离
                    dist = math.sqrt((current_point[0] - neighbor[0])**2 + 
                                      (current_point[1] - neighbor[1])**2)
                    
                    # 如果距离小于阈值，将该点加入当前簇
                    if dist < self.clustering_distance:
                        current_cluster.append(neighbor)
                        unassigned_points.pop(j)
                    else:
                        j += 1
            
            # 只有当簇包含足够多的点时才认为是障碍物
            if len(current_cluster) >= self.min_obstacle_points:
                # 计算簇的中心点
                center_x = sum(p[0] for p in current_cluster) / len(current_cluster)
                center_y = sum(p[1] for p in current_cluster) / len(current_cluster)
                min_distance = min(p[2] for p in current_cluster)  # 最小距离
                
                # 计算障碍物半径：找出离中心最远的点的距离
                max_dist_from_center = 0.0
                for point in current_cluster:
                    dist_from_center = math.sqrt((point[0] - center_x)**2 + (point[1] - center_y)**2)
                    if dist_from_center > max_dist_from_center:
                        max_dist_from_center = dist_from_center
                
                # 限制半径在合理范围内
                radius = max(self.min_obstacle_radius, min(max_dist_from_center, self.max_obstacle_radius))
                
                # 添加到障碍物列表
                clusters.append({
                    'x': center_x,
                    'y': center_y,
                    'radius': radius,
                    'distance': min_distance,
                    'points': len(current_cluster)
                })
                
                # 调试输出障碍物信息
                rospy.loginfo_throttle(1.0, f"检测到障碍物: 中心=({center_x:.2f}, {center_y:.2f}), 半径={radius:.2f}m, 点数={len(current_cluster)}")
        
        return clusters
    
    def publish_obstacles_visualization(self):
        """发布障碍物可视化"""
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(self.obstacles):
            # 创建障碍物标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = obstacle['radius'] * 2
            marker.scale.y = obstacle['radius'] * 2
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker.pose.position.x = obstacle['x']
            marker.pose.position.y = obstacle['y']
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(0.5)  # 0.5秒
            
            marker_array.markers.append(marker)
            
            # 添加文本标记显示距离
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "obstacle_texts"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.2  # 文本高度
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.8
            text_marker.pose.position.x = obstacle['x']
            text_marker.pose.position.y = obstacle['y']
            text_marker.pose.position.z = 0.6
            text_marker.text = f"{obstacle['distance']:.2f}m"
            text_marker.lifetime = rospy.Duration(0.5)  # 0.5秒
            
            marker_array.markers.append(text_marker)
        
        self.obstacles_marker_pub.publish(marker_array)
    
    def check_obstacles(self):
        """检查是否有障碍物在路径上并确定最佳避障方向"""
        # 处理点云数据，获取障碍物
        self.process_pointcloud()
        
        if not self.obstacles or not self.odom_data:
            # 如果没有障碍物数据，但之前检测到障碍物，则重置状态
            if self.obstacle_detected:
                rospy.loginfo("障碍物数据为空，重置检测状态")
                self.obstacle_detected = False
                self.obstacle_avoidance_active = False
                self.preferred_turn_direction = None  # 重置优先转向方向
                self.publish_obstacle_status(0)  # 0表示无障碍物
            return False
            
        # 获取机器人当前位置和朝向
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # 记录是否检测到前方有障碍物
        obstacle_in_front = False
        closest_obstacle = None
        closest_distance = float('inf')
        
        # 计算目标方向（如果有目标）
        goal_direction = None
        if self.goal_x is not None and self.goal_y is not None:
            goal_angle = math.atan2(self.goal_y - robot_y, self.goal_x - robot_x)
            goal_direction = self.normalize_angle(goal_angle - yaw)
        
        # 在这里，我们检查是否有障碍物在机器人前方的路径上
        for obstacle in self.obstacles:
            # 计算障碍物相对于机器人的距离和角度
            dx = obstacle['x'] - robot_x
            dy = obstacle['y'] - robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # 计算障碍物相对于机器人的角度
            angle = math.atan2(dy, dx)
            relative_angle = self.normalize_angle(angle - yaw)
            angle_diff = abs(relative_angle)
            
            # 定期打印调试信息
            now = rospy.Time.now()
            if self.debug and (now - self.last_debug_time) > self.debug_interval:
                self.last_debug_time = now
                rospy.loginfo(f"障碍物检测: 距离={distance:.2f}m, 角度差={angle_diff:.2f}rad, 相对角度={relative_angle:.2f}rad")
                rospy.loginfo(f"机器人位置: ({robot_x:.2f}, {robot_y:.2f}), 障碍物: ({obstacle['x']:.2f}, {obstacle['y']:.2f}), 半径: {obstacle['radius']:.2f}m")
            
            # 只在前方扇区检测障碍物，和安全距离内
            if distance < (obstacle['radius'] + self.safety_margin + self.robot_radius + self.detection_range) and angle_diff < math.pi / 3:
                obstacle_in_front = True
                
                # 记录最近的障碍物
                if distance < closest_distance:
                    closest_distance = distance
                    closest_obstacle = obstacle
                    closest_obstacle['relative_angle'] = relative_angle  # 记录相对角度用于确定转向方向
        
        # 如果找到了障碍物，确定最佳避障方向
        if obstacle_in_front and closest_obstacle:
            if not self.obstacle_detected:
                # 新检测到障碍物，确定避障方向
                relative_angle = closest_obstacle['relative_angle']
                
                # 确定最佳避障方向（根据障碍物位置和目标位置）
                if self.preferred_turn_direction is None:
                    # 如果障碍物在左侧，则向右转避障；如果在右侧，则向左转避障
                    suggested_direction = 1 if relative_angle < 0 else -1
                    
                    # 考虑目标位置的影响
                    if goal_direction is not None:
                        # 如果目标在障碍物的一侧，优先选择朝目标方向绕行
                        if (goal_direction < 0 and relative_angle > 0) or (goal_direction > 0 and relative_angle < 0):
                            suggested_direction = -1 if relative_angle > 0 else 1
                    
                    self.preferred_turn_direction = suggested_direction
                    rospy.loginfo(f"确定避障方向: {'右' if self.preferred_turn_direction > 0 else '左'}, 障碍物相对角度: {relative_angle:.2f}rad")
                
                rospy.loginfo(f"检测到障碍物: 距离={closest_distance:.2f}m, 角度差={abs(relative_angle):.2f}rad")
                self.obstacle_detected = True
                self.obstacle_avoidance_active = True
                self.last_obstacle = closest_obstacle
                self.publish_obstacle_status(1)  # 1表示检测到障碍物
        
        # 如果前方没有障碍物但是标志位还是True，需要重置状态
        elif not obstacle_in_front and self.obstacle_detected:
            rospy.loginfo("已避开障碍物")
            self.obstacle_detected = False
            self.obstacle_avoidance_active = False
            self.preferred_turn_direction = None  # 重置优先转向方向
            self.publish_obstacle_status(0)  # 0表示无障碍物
            return False
        
        return obstacle_in_front
        
    def publish_obstacle_status(self, status):
        """发布障碍物状态信息"""
        msg = Point()
        msg.x = float(status)  # 0: 无障碍物, 1: 有障碍物
        msg.y = float(self.obstacle_avoidance_active)  # 避障是否激活
        self.obstacle_status_pub.publish(msg)
        
        # 添加日志
        if self.debug:
            rospy.loginfo(f"发布障碍物状态: 检测={status}, 避障激活={self.obstacle_avoidance_active}")
        
    def normalize_angle(self, angle):
        """归一化角度到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
        
    def predict_motion(self, speed, turn_rate):
        """预测机器人在给定速度和转向率下的运动轨迹"""
        if self.odom_data is None:
            return []

        x = self.odom_data.pose.pose.position.x
        y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        path = []
        for i in range(100):
            yaw += turn_rate * self.step_time
            x += speed * math.cos(yaw) * self.step_time
            y += speed * math.sin(yaw) * self.step_time
            path.append((x, y))

        return path

    def check_for_collisions(self, path):
        """检查路径是否与障碍物碰撞，并返回更精细的风险评分"""
        if not self.obstacles:
            return 0
            
        min_distance = float('inf')  # 记录路径与障碍物的最小距离
        collision_detected = False
        
        for i, (x, y) in enumerate(path):
            # 每个路径点的风险随着路径长度增加而降低（远处的碰撞风险影响较小）
            path_weight = 1.0 - 0.2 * (i / len(path))  # 路径权重从1.0递减到0.5
            
            # 检查与每个障碍物的碰撞
            for obstacle in self.obstacles:
                # 计算路径点到障碍物的距离
                dx = x - obstacle['x']
                dy = y - obstacle['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                # 更新最小距离
                if distance < min_distance:
                    min_distance = distance
                
                # 如果距离小于障碍物半径加安全裕度和机器人半径，判定为碰撞
                if distance < (obstacle['radius'] + self.safety_margin + self.robot_radius):
                    collision_detected = True
                    return -100000 * path_weight  # 碰撞惩罚，越近的路径点权重越高
        
        # 如果没有碰撞，则根据最小距离返回一个柔和的风险评分
        if min_distance < float('inf'):
            # 使用最大障碍物半径进行评分，确保与所有障碍物保持足够距离
            max_obstacle_radius = max([obstacle['radius'] for obstacle in self.obstacles])
            if min_distance < (max_obstacle_radius + self.safety_margin + self.robot_radius + 0.3):
                return -5 * (1.0 - (min_distance / (max_obstacle_radius + self.safety_margin + self.robot_radius + 0.3)))
        
        return 0  # 无碰撞风险

    def choose_best_path(self, possible_paths):
        """选择最佳路径，优化评分规则以确保速度平滑且方向更一致"""
        if self.odom_data is None:
            return self.min_speed, 0.0
        
        if self.goal_x is None or self.goal_y is None:
            rospy.logwarn("目标坐标未设置，等待路径...")
            return self.min_speed, 0.0

        current_x = self.odom_data.pose.pose.position.x
        current_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        distance_to_goal = math.hypot(self.goal_x - current_x, self.goal_y - current_y)
        if distance_to_goal < 0.1:  # 增加目标到达阈值，避免频繁停止
            if not self.goal_reached:
                self.goal_reached = True
                rospy.loginfo(f"Goal reached at ({self.goal_x}, {self.goal_y})!")
            return self.min_speed, 0.0  # 返回最小速度，确保平稳减速

        best_score = float('-inf')
        best_speed, best_turn = self.min_speed, 0.0

        # 获取当前速度和角速度，用于平滑计算
        current_speed = self.last_cmd_vel['linear']
        current_turn = self.last_cmd_vel['angular']

        for speed, turn, path in possible_paths:
            # 计算路径终点与目标的距离
            goal_distance_score = -math.hypot(path[-1][0] - self.goal_x, path[-1][1] - self.goal_y) * 3
            
            # 计算朝向目标的偏差
            angle_to_goal = math.atan2(self.goal_y - current_y, self.goal_x - current_x)
            heading_diff = abs(self.normalize_angle(angle_to_goal - yaw))
            heading_score = -heading_diff * 5
            
            # 检查碰撞风险
            collision_risk = self.check_for_collisions(path)
            
            # 平滑度评分 - 降低与当前速度差异太大的路径得分
            speed_change = abs(speed - current_speed)
            turn_change = abs(turn - current_turn)
            smoothness_score = -3.0 * speed_change - 5.0 * turn_change
            
            # 速度奖励（鼓励更高的速度）- 适当提高速度权重
            speed_reward = speed * 2
            
            # 避障方向奖励（如果有优先避障方向）
            direction_bonus = 0
            if self.preferred_turn_direction is not None:
                # 如果转向方向与优先方向一致，给予奖励
                if (self.preferred_turn_direction > 0 and turn > 0) or (self.preferred_turn_direction < 0 and turn < 0):
                    direction_match = 1.0 - abs(abs(turn) - 0.3) / 0.3  # 转向率接近0.3时得分最高
                    direction_bonus = 15 * direction_match  # 提高方向一致性的权重
            
            # TODO：：总分计算
            total_score = (goal_distance_score + heading_score + collision_risk + smoothness_score + speed_reward + direction_bonus)
            # 如果得分更高，则更新最佳路径
            if total_score > best_score:
                best_score = total_score
                best_speed, best_turn = speed, turn
        
        # 确保返回的速度不小于最小速度，即使最佳路径的分数很低
        if best_speed < self.min_speed:
            best_speed = self.min_speed
        
        if self.debug and (rospy.Time.now() - self.last_debug_time) > self.debug_interval:
            rospy.loginfo(f"选择最佳路径: vx={best_speed:.2f}, vyaw={best_turn:.2f}, 分数={best_score:.2f}")
        
        return best_speed, best_turn

    def generate_paths(self):
        """生成可能的路径样本，考虑优先避障方向，并确保有足够多的前进路径"""
        possible_paths = []
        
        # 根据是否有优先转向方向，调整路径生成策略
        if self.preferred_turn_direction is not None:
            # 有优先转向方向时，60%的路径按照优先方向生成
            # 30%的路径是直行路径
            # 10%的路径是随机生成
            
            # 确保最小速度不为零
            min_speed = max(self.min_speed, 0.12)
            
            # 60%按照优先方向生成
            for i in range(60):
                # 确保速度在最小值和最大值之间，且分布更均匀
                speed = min_speed + (self.max_speed - min_speed) * (i / 60.0)
                # 根据优先方向生成转向角，确保转向角在正确的方向
                if self.preferred_turn_direction > 0:  # 右转优先
                    turn = random.uniform(0.05, self.max_turn)
                else:  # 左转优先
                    turn = random.uniform(-self.max_turn, -0.05)
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
            
            # 30%直行路径，让机器人可以在合适的情况下直行通过
            for i in range(30):
                # 确保直行路径有不同的速度
                speed = min_speed + (self.max_speed - min_speed) * (i / 30.0) 
                turn = random.uniform(-0.05, 0.05)  # 接近直线的小转向
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
            
            # 10%随机路径，以防万一上述策略不适用
            for i in range(10):
                speed = random.uniform(min_speed, self.max_speed)
                turn = random.uniform(-self.max_turn, self.max_turn)
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
                
        else:
            # 没有优先转向方向时，生成均匀分布的路径
            # 确保最小速度不为零
            min_speed = max(self.min_speed, 0.12)
            
            for i in range(100):
                # 确保速度在最小值和最大值之间，且分布更均匀
                speed = min_speed + (self.max_speed - min_speed) * (i / 100.0)
                turn = random.uniform(-self.max_turn, self.max_turn)
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
                
        return possible_paths 

    def find_closest_waypoint(self):
        """找到距离当前位置最近的路径点，同时考虑与所有障碍物的距离"""
        if not self.has_path or not self.odom_data:
            return None
            
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        
        min_dist = float('inf')
        closest_idx = self.current_waypoint_index
        found_safe_waypoint = False
        min_safe_distance = 1.0  # 设置安全距离为1米
        
        # 从当前索引开始查找最近的路径点
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            dist = math.hypot(
                waypoint.pose.position.x - robot_x,
                waypoint.pose.position.y - robot_y
            )
            
            # 检查此路径点是否与所有障碍物都保持至少1米的距离
            is_safe = True
            if self.obstacles:
                for obstacle in self.obstacles:
                    obstacle_dist = math.hypot(
                        waypoint.pose.position.x - obstacle['x'],
                        waypoint.pose.position.y - obstacle['y']
                    )
                    
                    # 如果路径点距离任何障碍物太近（小于1米），标记为不安全
                    if obstacle_dist < min_safe_distance:
                        is_safe = False
                        break
            
            # 只考虑安全的路径点
            if is_safe:
                found_safe_waypoint = True
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
        
        # 如果没有找到安全的路径点，直接返回最后一个路径点
        if self.obstacles and not found_safe_waypoint:
            rospy.loginfo("没有找到距离所有障碍物至少1米的路径点，切换到最后一个路径点")
            return self.waypoints[-1]
        
        # 如果找到的点就是当前点，尝试前进以避开障碍物
        if closest_idx == self.current_waypoint_index and closest_idx + 3 < len(self.waypoints):
            # 检查前方的点是否安全
            for i in range(closest_idx + 3, len(self.waypoints)):
                waypoint = self.waypoints[i]
                is_safe = True
                
                # 检查此前方路径点是否安全
                if self.obstacles:
                    for obstacle in self.obstacles:
                        obstacle_dist = math.hypot(
                            waypoint.pose.position.x - obstacle['x'],
                            waypoint.pose.position.y - obstacle['y']
                        )
                        if obstacle_dist < min_safe_distance:
                            is_safe = False
                            break
                
                if is_safe:
                    rospy.loginfo(f"使用前方安全路径点：索引 {i}，距离当前 {i - closest_idx} 个点")
                    return self.waypoints[i]
            
            # 如果前方没有安全点，返回最后一个点
            rospy.loginfo("前方没有安全路径点，切换到最后一个路径点")
            return self.waypoints[-1]
        
        return self.waypoints[closest_idx]

    def movement_loop(self, event):
        """主运动控制循环，优化平滑控制"""
        if self.odom_data is None:
            rospy.logwarn_throttle(1, "odom_data is None, waiting for odometry data...")
            return
            
        if self.pointcloud_data is None:
            rospy.logwarn_throttle(1, "pointcloud_data is None, waiting for point cloud data...")
            return
        
        # 修改：首先检查是否有有效路径，如果没有则直接返回，不发送任何控制命令
        if not self.has_path or self.goal_reached:
            # 如果之前在避障模式，先发布避障状态已清除
            if self.obstacle_avoidance_active:
                rospy.loginfo("退出DWA避障模式")
                self.obstacle_avoidance_active = False
                self.preferred_turn_direction = None
                self.publish_obstacle_status(0)
                
                # 发送一个停止命令以确保机器人停止
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
                
                # 重置速度记录
                self.last_cmd_vel['linear'] = 0.0
                self.last_cmd_vel['angular'] = 0.0
            
            # 不发送任何控制命令就直接返回
            return
        
        # 检查路径是否已完成（到达最后一个路径点）
        if self.current_waypoint_index >= len(self.waypoints):
            # 路径已完成
            if not self.goal_reached:
                rospy.loginfo("DWA路径跟踪完成")
                self.goal_reached = True
                
                # 发送一个停止命令
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
                
                # 重置速度记录
                self.last_cmd_vel['linear'] = 0.0
                self.last_cmd_vel['angular'] = 0.0
                
                # 如果之前在避障模式，发布避障状态已清除
                if self.obstacle_avoidance_active:
                    self.obstacle_avoidance_active = False
                    self.preferred_turn_direction = None
                    self.publish_obstacle_status(0)
            
            return
            
        # 检查是否有障碍物
        obstacle_present = self.check_obstacles()
        
        # 生成候选路径
        possible_paths = self.generate_paths()
        
        # 根据是否有障碍物决定控制模式
        # if False:
        if obstacle_present or self.obstacle_avoidance_active:
            # 障碍物避障模式
            if not self.obstacle_avoidance_active:
                rospy.loginfo("启动DWA避障模式") 
                self.obstacle_avoidance_active = True
                self.publish_obstacle_status(1) 
                
            # 使用DWA算法选择最佳路径
            speed, turn = self.choose_best_path(possible_paths)
            
            # 确保最小速度不为零，保持避障平滑
            if speed < self.min_speed:
                speed = self.min_speed
                
            # 如果转向角过大，适当降低速度以保证安全，但不低于最小速度
            if abs(turn) > 0.4:
                # 转向角越大，速度越低，但不低于最小速度
                speed = max(self.min_speed, speed * (1.0 - (abs(turn) - 0.4) / 0.6))
            
            # 平滑速度变化
            smooth_speed = self.last_cmd_vel['linear'] + self.smooth_factor * (speed - self.last_cmd_vel['linear'])
            smooth_turn = self.last_cmd_vel['angular'] + self.smooth_factor * (turn - self.last_cmd_vel['angular'])
            
            # 发布控制命令
            move_cmd = Twist()
            move_cmd.linear.x = smooth_speed
            move_cmd.angular.z = smooth_turn
            self.cmd_publisher.publish(move_cmd)
            
            # 更新上一次的速度记录
            self.last_cmd_vel['linear'] = smooth_speed
            self.last_cmd_vel['angular'] = smooth_turn
            
            if self.debug and (rospy.Time.now() - self.last_debug_time) > self.debug_interval:
                rospy.loginfo(f"DWA避障控制: vx={smooth_speed:.2f}, vyaw={smooth_turn:.2f}, 方向={'右' if self.preferred_turn_direction > 0 else '左' if self.preferred_turn_direction < 0 else '无'}")
            
            # 发布可视化路径
            self.publish_path_visualization(possible_paths)
            
        else:
            # 如果刚刚从避障模式切换回来，找到最近的路径点
            if self.obstacle_avoidance_active:
                rospy.loginfo("障碍物已清除")
                rospy.loginfo("退出DWA避障模式，恢复A*路径跟踪")
                self.obstacle_avoidance_active = False
                self.preferred_turn_direction = None
                self.publish_obstacle_status(0)
                
                # 找到最近的路径点
                if self.has_path and len(self.waypoints) > 0:
                    closest_waypoint = self.find_closest_waypoint()
                    if closest_waypoint:
                        new_index = self.waypoints.index(closest_waypoint)
                        old_index = self.current_waypoint_index
                        if new_index > self.current_waypoint_index:
                            rospy.loginfo(f"避障后重新定位到路径点 {new_index} (原路径点: {old_index}，总点数: {len(self.waypoints)})")
                            self.current_waypoint_index = new_index
            
            # 正常模式，发送零速度，让A*路径控制接管
            # 但是，为了保持平滑过渡，我们逐渐将速度降低到0，而不是突然停止
            if self.last_cmd_vel['linear'] > 0.01 or abs(self.last_cmd_vel['angular']) > 0.01:
                # 平滑减速
                new_linear = self.last_cmd_vel['linear'] * 0.5  # 每次减少50%
                new_angular = self.last_cmd_vel['angular'] * 0.5  # 每次减少50%

                print(f"平滑减速: new_linear={new_linear:.2f}, new_angular={new_angular:.2f}")
                
                # 如果速度非常小，直接设为0
                if new_linear < 0.01: 
                    new_linear = 0.0
                if abs(new_angular) < 0.01:
                    new_angular = 0.0
                    
                # 发送逐渐减小的速度
                move_cmd = Twist()
                move_cmd.linear.x = new_linear
                move_cmd.angular.z = new_angular
                self.cmd_publisher.publish(move_cmd)
                
                # 更新上一次的速度记录
                self.last_cmd_vel['linear'] = new_linear
                self.last_cmd_vel['angular'] = new_angular
            else:
                # 完全停止
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
                
                # 重置速度记录
                self.last_cmd_vel['linear'] = 0.0
                self.last_cmd_vel['angular'] = 0.0

            
    def publish_path_visualization(self, possible_paths):
        """发布路径可视化标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.a = 1.0
        
        for _, _, path in possible_paths:
            for x, y in path:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                marker.points.append(point)
                
        self.path_publisher.publish(marker)
        
    def publish_filtered_points(self, points):
        """发布过滤后的点云可视化"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for x, y, _ in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)
        
        self.filtered_points_pub.publish(marker)
        
    def run(self):
        """运行DWA规划器"""
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = DWAPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass # Test
