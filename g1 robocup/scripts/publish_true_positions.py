#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
import numpy as np

class TruePositionPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('true_position_publisher', anonymous=True)
        
        # 发布者
        self.ball_pub = rospy.Publisher('/ball_true_position', Point, queue_size=10)
        self.left_post_pub = rospy.Publisher('/left_goalpost_true_position', Point, queue_size=10)
        self.right_post_pub = rospy.Publisher('/right_goalpost_true_position', Point, queue_size=10)
        
        # 位置设置（场地坐标系）
        # 这些值可以根据实际情况调整或从Gazebo中获取
        self.ball_position = Point()
        self.ball_position.x = 3.0    # 场地中心位置
        self.ball_position.y = 0.0
        self.ball_position.z = 0.3    # 球半径约10cm
        
        self.left_post_position = Point()
        self.left_post_position.x = 4.5    # 右球门左门柱
        self.left_post_position.y = 1.4
        self.left_post_position.z = 0.75   # 门柱高度
        
        self.right_post_position = Point()
        self.right_post_position.x = 4.5   # 右球门右门柱
        self.right_post_position.y = -1.4
        self.right_post_position.z = 0.75  # 门柱高度
        
        # 启动定时器，定期发布位置信息
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 10Hz
        
        rospy.loginfo("真值位置发布器已启动")
    
    def timer_callback(self, event):
        """定时发布位置信息"""
        # 发布球的位置
        self.ball_pub.publish(self.ball_position)
        
        # 发布门柱位置
        self.left_post_pub.publish(self.left_post_position)
        self.right_post_pub.publish(self.right_post_position)
    
    def run(self):
        """主循环"""
        rospy.spin()

if __name__ == "__main__":
    try:
        publisher = TruePositionPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 