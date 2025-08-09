#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose2D
from unitree_legged_msgs.msg import MoveCmd
import numpy as np
import math
import time

class VisualServoBallKicker:
    def __init__(self):
        rospy.init_node("visual_servo_ball_kicker", anonymous=True)

        # 状态变量
        self.robot_pose_ = None  # Pose2D
        self.ball_pose_ = None   # Pose2D
        self.goal_pose_ = None   # Pose2D

        # 控制参数
        self.position_tolerance_ = 0.15      # 位置容差 (m)
        self.kick_distance_ = 0.15           # 踢球判定距离 (m)
        self.max_arena_bound_ = 4.5          # 最大允许X坐标，防止走出场地

        # 发布控制指令
        self.cmd_pub_ = rospy.Publisher('/g1_12dof_gazebo/move_controller/command', MoveCmd, queue_size=10)

        # 订阅定位信息
        rospy.Subscriber('/global_position_2d_robot', Pose2D, self.robot_callback)
        rospy.Subscriber('/global_position_2d_ball', Pose2D, self.ball_callback)
        rospy.Subscriber('/global_position_2d_goal', Pose2D, self.goal_callback)

        rospy.loginfo("[视觉伺服] 初始化完成，等待定位信息...")
        self.control_loop()

    def robot_callback(self, msg):
        self.robot_pose_ = msg

    def ball_callback(self, msg):
        self.ball_pose_ = msg

    def goal_callback(self, msg):
        self.goal_pose_ = msg

    def control_loop(self):
        rate = rospy.Rate(10)
        state = "SEARCHING"

        while not rospy.is_shutdown():
            if self.robot_pose_ and self.robot_pose_.x > self.max_arena_bound_:
                rospy.logwarn("[安全保护] 机器人位置超出边界 (x=%.2f)，强制停止", self.robot_pose_.x)
                self.stop_robot()
                rate.sleep()
                continue

            if self.robot_pose_ and self.ball_pose_ and self.goal_pose_:
                rospy.loginfo_throttle(1, "[调试] Robot(%.2f, %.2f, %.2f)  Ball(%.2f, %.2f)  Goal(%.2f, %.2f)",
                    self.robot_pose_.x, self.robot_pose_.y, self.robot_pose_.theta,
                    self.ball_pose_.x, self.ball_pose_.y,
                    self.goal_pose_.x, self.goal_pose_.y)

            if self.ball_pose_ is None or self.goal_pose_ is None:
                rospy.loginfo_throttle(1, "[视觉伺服] 未检测到球或门，执行搜索旋转")
                cmd = MoveCmd()
                cmd.vyaw = 0.3
                self.cmd_pub_.publish(cmd)
                rate.sleep()
                continue

            # 计算球与机器人的相对方向和距离
            ball_vec = np.array([self.ball_pose_.x, self.ball_pose_.y]) - np.array([self.robot_pose_.x, self.robot_pose_.y])
            ball_dist = np.linalg.norm(ball_vec)
            ball_yaw = math.atan2(ball_vec[1], ball_vec[0])
            yaw_error = ball_yaw - self.robot_pose_.theta
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

            if state == "SEARCHING":
                if self.ball_pose_ and self.goal_pose_:
                    state = "APPROACH"
                    rospy.loginfo("[视觉伺服] 目标已检测，开始逼近")
                else:
                    rate.sleep()
                    continue

            elif state == "APPROACH":
                if ball_dist < self.kick_distance_:
                    self.stop_robot()
                    rospy.loginfo("[视觉伺服] 已靠近球，准备踢球")
                    state = "KICK"
                    continue

                cmd = MoveCmd()
                cmd.vyaw = np.clip(-0.5 * yaw_error, -0.3, 0.3)
                cmd.vx = 0.0
                cmd.vy = 0.3 * math.cos(yaw_error)  # 方向偏差越小前进越快
                self.cmd_pub_.publish(cmd)
                rospy.loginfo_throttle(1, "[视觉伺服] 逼近中 dist=%.2f yaw=%.2f", ball_dist, yaw_error)

            elif state == "KICK":
                cmd = MoveCmd()
                cmd.vx = 0.0
                cmd.vy = 1.0
                cmd.vyaw = 0.0
                self.cmd_pub_.publish(cmd)
                rospy.loginfo("[视觉伺服] 踢球中...")
                rospy.sleep(2.0)
                self.stop_robot()
                rospy.loginfo("[视觉伺服] 踢球完成")
                state = "DONE"

            elif state == "DONE":
                rospy.loginfo_throttle(5, "[视觉伺服] 任务完成")

            rate.sleep()

    def stop_robot(self):
        for _ in range(5):
            cmd = MoveCmd()
            cmd.vx = 0.0
            cmd.vy = 0.0
            cmd.vyaw = 0.0
            self.cmd_pub_.publish(cmd)
            rospy.sleep(0.02)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        VisualServoBallKicker()
    except rospy.ROSInterruptException:
        pass
