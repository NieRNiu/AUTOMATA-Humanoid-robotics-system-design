#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose2D
import numpy as np
from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from unitree_legged_msgs.msg import MoveCmd
import math
from enum import Enum
import time

class RobotState(Enum):
    INIT = 0                    # 初始化状态
    ROTATE_FOR_DETECTION = 1    # 旋转以获取检测结果
    WAIT_FOR_DETECTION = 2      # 等待YOLO检测结果
    NAVIGATE_TO_BALL = 3        # 移动到球前方
    ADJUST_ORIENTATION = 4      # 调整朝向
    KICK_BALL = 5               # 踢球
    DONE = 6                    # 完成

class BallKickerYolo:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ball_kicker_yolo', anonymous=True)
        
        # 成员变量 - 位置和姿态信息
        self.robot_position_ = None          # 机器人位置 (Point)
        self.robot_orientation_ = None       # 机器人姿态 (Quaternion)
        self.ball_position_ = None           # 足球位置 (检测值)
        self.goal_position_ = None           # 球门位置 (检测值)
        self.robot_global_position_ = None   # 机器人全局位置 (Pose2D)
        
        # 任务状态
        self.state_ = RobotState.INIT
        self.kick_position_ = None           # 踢球位置 (计算值)
        self.kick_direction_ = None          # 踢球方向 (计算值)
        
        # 配置参数 
        self.ball_offset_distance_ = 0.6     # 踢球位置与球的距离 (米)
        self.position_tolerance_ = 0.15      # 位置到达容差 (米)
        self.orientation_tolerance_ = 0.15   # 朝向到达容差 (弧度)
        self.kick_distance_ = 0.1            # 踢球距离 (米)
        self.detection_timeout_ = 5.0        # 检测超时时间 (秒)
        self.detection_start_time_ = None    # 检测开始时间
        
        # ROS订阅者 - 使用机器人位置和姿态
        rospy.Subscriber('/g1_12dof_gazebo/position', Point, self.position_callback)
        rospy.Subscriber('/g1_12dof_gazebo/orientation', Quaternion, self.orientation_callback)
        
        # ROS订阅者 - 使用YOLO检测结果
        rospy.Subscriber('/global_position_2d_robot', Pose2D, self.robot_global_position_callback)
        rospy.Subscriber('/global_position_2d_ball', Pose2D, self.ball_position_callback)
        rospy.Subscriber('/global_position_2d_goal', Pose2D, self.goal_position_callback)
        
        # ROS发布者
        self.cmd_vel_pub_ = rospy.Publisher('/g1_12dof_gazebo/move_controller/command', MoveCmd, queue_size=10)
        self.target_pub_ = rospy.Publisher('/navigation_target', PoseStamped, queue_size=10)
        self.kick_pos_pub_ = rospy.Publisher('/kick_position', PoseStamped, queue_size=10)
        
        rospy.loginfo("[初始化] 基于YOLO检测的球踢球导航系统已启动")
        
        # 主循环
        self.navigate()
    
    def position_callback(self, position):
        """处理机器人位置信息"""
        self.robot_position_ = position
    
    def orientation_callback(self, orientation):
        """处理机器人姿态信息"""
        self.robot_orientation_ = orientation
    
    def robot_global_position_callback(self, pose2d):
        """处理机器人全局位置信息"""
        self.robot_global_position_ = pose2d
        rospy.loginfo("[全局位置] 获取到机器人全局位置: (%.2f, %.2f, %.2f)", 
                     pose2d.x, pose2d.y, pose2d.theta)
    
    def ball_position_callback(self, pose2d):
        """处理足球位置信息"""
        if self.ball_position_ is None:
            rospy.loginfo("[球位置] 获取到足球全局位置: (%.2f, %.2f)", 
                         pose2d.x, pose2d.y)
            self.ball_position_ = pose2d
    
    def goal_position_callback(self, pose2d):
        """处理球门位置信息"""
        if self.goal_position_ is None:
            rospy.loginfo("[球门位置] 获取到球门全局位置: (%.2f, %.2f)", 
                         pose2d.x, pose2d.y)
            self.goal_position_ = pose2d
    
    def calculate_kick_position(self):
        """
        计算踢球位置和方向
        踢球位置: 在球和门柱中点连线上，位于球后方一定距离的点
        踢球方向: 从踢球位置指向球门中点的方向
        """
        if self.ball_position_ is None or self.goal_position_ is None:
            return None, None
        
        # 转换球的位置为numpy数组
        ball_pos = np.array([
            self.ball_position_.x,
            self.ball_position_.y,
            0.0  # 假设z=0
        ])
        
        # 转换球门的位置为numpy数组
        goal_pos = np.array([
            self.goal_position_.x,
            self.goal_position_.y,
            0.0  # 假设z=0
        ])
        
        # 计算从球指向球门中点的方向向量
        direction = goal_pos - ball_pos
        distance = np.linalg.norm(direction)
        if distance == 0:
            return None, None
        
        # 单位方向向量
        unit_direction = direction / distance
        
        # 计算踢球位置 (球后方一定距离)
        kick_pos = ball_pos - unit_direction * self.ball_offset_distance_
        
        rospy.loginfo("[踢球位置] 计算踢球位置: (%.2f, %.2f, %.2f)", 
                     kick_pos[0], kick_pos[1], kick_pos[2])
        
        # 将z坐标设为0 (确保在地面上)
        kick_pos[2] = 0.0
        
        # 发布踢球位置可视化
        self.publish_target_pose(kick_pos, unit_direction, self.kick_pos_pub_)
        
        return kick_pos, unit_direction
    
    def publish_target_pose(self, position, direction, publisher):
        """发布目标位姿用于可视化"""
        target_msg = PoseStamped()
        target_msg.header.frame_id = "world"
        target_msg.header.stamp = rospy.Time.now()
        
        # 设置位置
        target_msg.pose.position.x = position[0]
        target_msg.pose.position.y = position[1]
        target_msg.pose.position.z = position[2]
        
        # 计算朝向四元数 (将方向向量转换为四元数)
        yaw = math.atan2(direction[1], direction[0])
        q = quaternion_from_euler(0, 0, yaw)
        target_msg.pose.orientation.x = q[0]
        target_msg.pose.orientation.y = q[1]
        target_msg.pose.orientation.z = q[2]
        target_msg.pose.orientation.w = q[3]
        
        publisher.publish(target_msg)
    
    def get_robot_yaw(self):
        """获取机器人当前的偏航角"""
        if self.robot_orientation_ is None:
            return None
        
        q = [
            self.robot_orientation_.x,
            self.robot_orientation_.y,
            self.robot_orientation_.z,
            self.robot_orientation_.w
        ]
        _, _, yaw = euler_from_quaternion(q)
        return yaw
    
    def move_to(self, target_pos):
        """控制机器人移动到目标位置"""
        if self.robot_global_position_ is None or target_pos is None:
            return False
        
        # 当前机器人位置
        current_pos = np.array([
            self.robot_global_position_.x,
            self.robot_global_position_.y,
            0.0
        ])
        
        # 打印目标位置信息
        rospy.loginfo("[移动] 目标位置: (%.2f, %.2f, %.2f)", target_pos[0], target_pos[1], target_pos[2])
        
        # 计算目标方向和距离
        delta = target_pos - current_pos
        distance = np.linalg.norm(delta[:2])  # 只考虑xy平面
        
        # 打印当前位置和距离信息
        rospy.loginfo("[移动] 当前位置: (%.2f, %.2f, %.2f), 距离目标: %.2f 米", 
                     current_pos[0], current_pos[1], current_pos[2], distance)
        
        if distance < self.position_tolerance_:  # 到达目标
            rospy.loginfo("[移动] 到达目标位置")
            self.stop_robot()
            return True
        
        # 计算目标朝向角度（偏航角）
        target_yaw = math.atan2(delta[1], delta[0])
        
        # 获取当前机器人的偏航角
        current_yaw = self.robot_global_position_.theta
        
        # 计算角度差（考虑角度的循环性）
        yaw_diff = target_yaw - current_yaw

        while yaw_diff > math.pi:
            yaw_diff -= 2*math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2*math.pi
        
        cmd = MoveCmd()
        
        # 如果角度差较大，先调整朝向
        if abs(yaw_diff) > self.orientation_tolerance_:
            # 角速度与角度差成比例
            cmd.vyaw = -np.sign(yaw_diff) * min(0.2, abs(yaw_diff))  # 角速度和角度差成比例，但限制最大值
            cmd.vy = 0.0  # 旋转时不前进
            cmd.vx = 0.0  # 不使用横向移动
            
            # 打印角度差信息
            rospy.loginfo("[移动] 调整朝向: 当前=%.2f, 目标=%.2f, 差值=%.2f, 角速度=%.2f", 
                         current_yaw, target_yaw, yaw_diff, cmd.vyaw)
        else:
            # 朝向基本正确，向前移动
            cmd.vyaw = -np.clip(yaw_diff, -0.2, 0.2)  # 微调朝向
            cmd.vy = min(0.5, distance)  # 速度与距离成比例，但限制最大值
            cmd.vx = 0.0  # 不使用横向移动
            
            rospy.loginfo("[移动] 前进: 速度=%.2f, 距离目标=%.2f", cmd.vy, distance)
        
        # 发送速度指令
        self.cmd_vel_pub_.publish(cmd)
        
        # 发布目标点用于可视化
        target_msg = PoseStamped()
        target_msg.header.frame_id = "world"
        target_msg.header.stamp = rospy.Time.now()
        target_msg.pose.position.x = target_pos[0]
        target_msg.pose.position.y = target_pos[1]
        target_msg.pose.position.z = target_pos[2]
        self.target_pub_.publish(target_msg)
        
        return False
    
    def turn_to_direction(self, target_yaw):
        """调整机器人朝向指定方向"""
        if self.robot_global_position_ is None:
            return False
        
        # 获取当前机器人的偏航角
        current_yaw = self.robot_global_position_.theta
        
        # 计算角度差 (考虑角度的循环性)
        yaw_diff = target_yaw - current_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2*math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2*math.pi
        
        # 如果角度差小于容差，认为已调整好
        if abs(yaw_diff) < self.orientation_tolerance_:
            rospy.loginfo("[调整朝向] 朝向已调整完成")
            self.stop_robot()
            return True
        
        # 发送旋转指令，角速度与角度差成比例
        cmd = MoveCmd()
        cmd.vx = 0.0
        cmd.vy = 0.0
        cmd.vyaw = -np.sign(yaw_diff) * min(0.2, abs(yaw_diff))
        
        # 发送速度指令
        self.cmd_vel_pub_.publish(cmd)
        
        rospy.loginfo("[调整朝向] 当前=%.2f, 目标=%.2f, 差值=%.2f, 角速度=%.2f", 
                     current_yaw, target_yaw, yaw_diff, cmd.vyaw)
        
        return False
    
    def adjust_orientation(self, target_direction):
        """调整机器人朝向指定方向"""
        if self.robot_global_position_ is None or target_direction is None:
            return False
        
        # 计算目标方向的偏航角
        target_yaw = math.atan2(target_direction[1], target_direction[0])
        
        return self.turn_to_direction(target_yaw)
    
    def kick_ball_action(self):
        """执行踢球动作 (向前直走)"""
        if self.robot_global_position_ is None or self.goal_position_ is None or self.ball_position_ is None:
            return False
        
        # 当前机器人位置
        current_pos = np.array([
            self.robot_global_position_.x,
            self.robot_global_position_.y,
            0.0
        ])
        rospy.loginfo("机器人位置: (%.2f, %.2f)",self.robot_global_position_.x,self.robot_global_position_.y)
        
        # 计算到目标(球门中点)的距离
        goal_pos = np.array([
            self.goal_position_.x,
            self.goal_position_.y,
            0.0
        ])
        
        delta = goal_pos - current_pos
        distance = np.linalg.norm(delta[:2])  # 只考虑xy平面
        
        # 如果已经足够靠近球门，认为踢球完成
        if distance < self.kick_distance_:
            rospy.loginfo("[踢球动作] 踢球完成，距离球门: %.2f m", distance)
            self.stop_robot()
            return True
        
        # 计算横向补偿值 - 基于球和球门中心的相对位置
        ball_pos_y = self.ball_position_.y
        goal_center_y = self.goal_position_.y
        lateral_offset = ball_pos_y - goal_center_y
        
        # 根据偏移量计算横向速度补偿 - 平滑过渡
        # 最大补偿值为0.3，随着球接近中心线补偿值线性减小
        max_compensation = 0.3
        max_offset = 1.0  # 当偏移达到1米时达到最大补偿值
        
        # 计算补偿值，确保在[-max_compensation, max_compensation]范围内
        vx_compensation = max_compensation * (lateral_offset / max_offset)
        vx_compensation = np.clip(vx_compensation, -max_compensation, max_compensation)
            
        rospy.loginfo("[踢球动作] 球位置Y=%.2f, 球门中心Y=%.2f, 横向偏移=%.2f, 横向补偿=%.2f", 
                     ball_pos_y, goal_center_y, lateral_offset, vx_compensation)
        
        # 发送向前直走的指令，速度可以与距离成比例
        cmd = MoveCmd()
        cmd.vy = 1.0  # 全速前进
        cmd.vx = vx_compensation  # 基于球位置的横向补偿
        cmd.vyaw = 0.0  # 不旋转
        self.cmd_vel_pub_.publish(cmd)
        
        rospy.loginfo("[踢球动作] 距离目标: %.2f m", distance)
        
        return False
    
    def stop_robot(self):
        """停止机器人运动"""
        # 发送多次停止命令确保生效
        for _ in range(5):
            cmd = MoveCmd()
            cmd.vx = 0.0
            cmd.vy = 0.0
            cmd.vyaw = 0.0
            self.cmd_vel_pub_.publish(cmd)
            time.sleep(0.02)  # 20ms间隔
        rospy.loginfo("[运动] 机器人已停止")
        time.sleep(0.1)  # 额外等待确保停止生效
    
    def navigate(self):
        """主导航逻辑"""
        rate = rospy.Rate(10)  # 10 Hz
        last_state = None
        
        while not rospy.is_shutdown():
            # 只在状态变化时打印，减少日志输出
            if self.state_ != last_state:
                rospy.loginfo("[导航] 状态切换: %s -> %s", last_state, self.state_)
                last_state = self.state_
            
            # 检查是否有所需的信息
            if self.state_ == RobotState.INIT:
                if self.robot_global_position_ is not None:
                    rospy.loginfo("[导航] 初始化完成，开始旋转以获取检测结果")
                    self.detection_start_time_ = rospy.Time.now()
                    self.state_ = RobotState.ROTATE_FOR_DETECTION
                else:
                    rospy.logdebug("[导航] 等待信息: 机器人全局位置")
            
            elif self.state_ == RobotState.ROTATE_FOR_DETECTION:
                # 缓慢旋转以获取检测结果
                cmd = MoveCmd()
                cmd.vx = 0.0
                cmd.vy = 0.0
                cmd.vyaw = 0.2  # 缓慢旋转
                self.cmd_vel_pub_.publish(cmd)
                
                # 检查是否已获取所有需要的信息
                if self.ball_position_ is not None and self.goal_position_ is not None:
                    rospy.loginfo("[导航] 已获取所有检测结果，停止旋转")
                    self.stop_robot()
                    self.state_ = RobotState.WAIT_FOR_DETECTION
                
                # 检查是否旋转超时（完整旋转一圈后仍未获取所有信息）
                elapsed_time = (rospy.Time.now() - self.detection_start_time_).to_sec()
                if elapsed_time > 15.0:  # 15秒足够旋转一圈
                    rospy.logwarn("[导航] 旋转超时，未获取到所有检测结果")
                    self.stop_robot()
                    # 重置计时器并继续等待
                    self.detection_start_time_ = rospy.Time.now()
                    self.state_ = RobotState.WAIT_FOR_DETECTION
            
            elif self.state_ == RobotState.WAIT_FOR_DETECTION:
                # 等待YOLO检测结果
                if self.ball_position_ is not None and self.goal_position_ is not None:
                    # 计算踢球位置和方向
                    self.kick_position_, self.kick_direction_ = self.calculate_kick_position()
                    
                    if self.kick_position_ is not None and self.kick_direction_ is not None:
                        rospy.loginfo("[导航] 检测完成，计算踢球位置: (%.2f, %.2f, %.2f)", 
                                     self.kick_position_[0], self.kick_position_[1], self.kick_position_[2])
                        self.state_ = RobotState.NAVIGATE_TO_BALL
                    else:
                        rospy.logwarn("[导航] 计算踢球位置失败，请检查球和球门位置")
                else:
                    # 检查是否超时
                    elapsed_time = (rospy.Time.now() - self.detection_start_time_).to_sec()
                    if elapsed_time > self.detection_timeout_:
                        rospy.logwarn("[导航] 检测超时，未获取到球或球门位置")
                        missing = []
                        if self.ball_position_ is None:
                            missing.append("球位置")
                        if self.goal_position_ is None:
                            missing.append("球门位置")
                        rospy.logwarn("[导航] 缺失信息: %s", ", ".join(missing))
                        
                        # 重新进入旋转状态
                        rospy.loginfo("[导航] 重新开始旋转以获取检测结果")
                        self.detection_start_time_ = rospy.Time.now()
                        self.state_ = RobotState.ROTATE_FOR_DETECTION
            
            elif self.state_ == RobotState.NAVIGATE_TO_BALL:
                # 更新踢球位置 (以应对球可能的移动)
                self.kick_position_, self.kick_direction_ = self.calculate_kick_position()
                
                if self.kick_position_ is not None:
                    if self.move_to(self.kick_position_):
                        rospy.loginfo("[导航] 到达踢球位置，开始调整朝向")
                        self.state_ = RobotState.ADJUST_ORIENTATION
                else:
                    rospy.logwarn("[导航] 踢球位置无效，重新计算")
            
            elif self.state_ == RobotState.ADJUST_ORIENTATION:
                # 更新踢球方向 
                _, self.kick_direction_ = self.calculate_kick_position()
                
                if self.kick_direction_ is not None:
                    if self.adjust_orientation(self.kick_direction_):
                        rospy.loginfo("[导航] 朝向调整完成，准备踢球")
                        self.state_ = RobotState.KICK_BALL
                else:
                    rospy.logwarn("[导航] 踢球方向无效，重新计算")
            elif self.state_ == RobotState.KICK_BALL:
                
                if self.kick_ball_action():
                    rospy.loginfo("[导航] 踢球完成")
                    self.state_ = RobotState.DONE
            
            elif self.state_ == RobotState.DONE:
                rospy.loginfo("[导航] 任务完成")
                break

            elif self.robot_global_position_.x > 4.3:
                rospy.loginfo("[导航]位置: (%.2f)",self.robot_global_position_.x)
                self.stop_robot()
            

            
            rate.sleep()
        
        rospy.loginfo("[导航] 导航系统关闭")

if __name__ == "__main__":
    try:
        kicker = BallKickerYolo()
    except rospy.ROSInterruptException:
        pass 
