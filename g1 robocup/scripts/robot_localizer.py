#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, Point  # 用于发布机器人位姿
from soccer_robot.msg import ObjectArray  # 更改为与robot_navigator.py一致的消息类型
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from itertools import combinations
from enum import Enum

# 添加YOLO和图像处理相关导入
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from soccer_robot.msg import Object
import os
from tf.transformations import euler_matrix
from geometry_msgs.msg import Point, Quaternion, Pose2D
import math
import logging
import time  # 添加时间模块用于推理时间统计

class CoordinateMode(Enum):
    """坐标计算模式枚举"""
    DEPTH_MODE = "depth_mode"          # 使用深度信息模式
    FIXED_Z_MODE = "fixed_z_mode"      # 固定z坐标模式

class CameraType(Enum):
    """相机类型枚举"""
    OVERHEAD_CAMERA = "overhead_camera"    # 俯视相机（倾斜相机，camera_id=1）
    HORIZONTAL_CAMERA = "horizontal_camera"  # 水平相机（camera_id=2）

class ExtendedObject:
    """扩展的Object类，包含相机坐标系和机器人坐标系信息"""
    def __init__(self, ros_object=None):
        if ros_object:
            self.type = ros_object.type
            self.x_robot_frame = ros_object.x
            self.y_robot_frame = ros_object.y
            self.z_robot_frame = ros_object.z
        else:
            self.type = ""
            self.x_robot_frame = 0.0
            self.y_robot_frame = 0.0
            self.z_robot_frame = 0.0
        
        # 相机坐标系坐标
        self.x_camera_frame = 0.0
        self.y_camera_frame = 0.0
        self.z_camera_frame = 0.0
        self.camera_id = 1
        
        # 地标标签（用于唯一标识地标）
        self.label = ""
        
        # RGB图像像素信息
        self.pixel_x = 0  # 检测框中心点的像素x坐标
        self.pixel_y = 0  # 检测框中心点的像素y坐标
        self.bbox_x1 = 0  # 检测框左上角x坐标
        self.bbox_y1 = 0  # 检测框左上角y坐标
        self.bbox_x2 = 0  # 检测框右下角x坐标
        self.bbox_y2 = 0  # 检测框右下角y坐标
        self.confidence = 0.0  # 检测置信度
        
        # 基于机器人真实位姿反算的真值坐标
        self.x_robot_frame_true = 0.0      # 机器人坐标系下真值
        self.y_robot_frame_true = 0.0
        self.z_robot_frame_true = 0.0
        self.x_camera_frame_true = 0.0        # 相机坐标系下真值
        self.y_camera_frame_true = 0.0
        self.z_camera_frame_true = 0.0
        self.has_true_values = False  # 标记是否计算了真值
    
    def to_ros_object(self):
        """转换为ROS Object消息"""
        obj = Object()
        obj.type = self.type
        obj.x = self.x_robot_frame
        obj.y = self.y_robot_frame
        obj.z = self.z_robot_frame
        return obj

class RobotLocalizer:
    def __init__(self):
        # 设置ROS日志等级为DEBUG，以便输出调试信息
        os.environ['ROSCONSOLE_SEVERITY_LEVEL'] = 'DEBUG'
        
        # 设置rospy的日志等级为DEBUG
        logging.getLogger('rosout').setLevel(logging.DEBUG)
        rospy.set_param('/rospy/logger_level', 'DEBUG')
        
        rospy.loginfo("已设置ROS日志等级为DEBUG，调试信息将会输出")
        rospy.logdebug("这是一条测试DEBUG信息 - 如果你看到这条信息说明DEBUG日志已启用")
        
        # 初始化CvBridge
        self.bridge_ = CvBridge()
        
        # 加载YOLO模型
        current_dir = os.path.dirname(os.path.abspath(__file__)) 
        model_path = os.path.join(current_dir, '../models/yolo11.pt')
        try:
            self.model_ = YOLO(model_path)
            rospy.loginfo("YOLO模型加载成功: %s", model_path)
        except Exception as e:
            rospy.logerr("加载YOLO模型失败: %s", e)
            raise

        # 相机位置和方向（相对于机器人坐标系）
        self.camera_offset_cam1_robot_frame_ = np.array([0.0576235, 0.01053, 0.41987])
        self.camera_orientation_cam1_ = np.array([0, 0.8307767239493009, 0])
        self.camera_offset_cam2_robot_frame_ = np.array([0, 0, 0.46987])
        self.camera_orientation_cam2_ = np.array([0, 0, 0])

        # D435i相机内参
        self.fx_ = 347.9975
        self.fy_ = 347.9975
        self.cx_ = 320
        self.cy_ = 240
        
        # 相机内参矩阵
        self.camera_matrix_ = np.array([[self.fx_, 0, self.cx_],
                                        [0, self.fy_, self.cy_],
                                        [0, 0, 1]])
        # 畸变系数（假设为0）
        self.dist_coeffs_ = np.zeros((4,1))

        # 坐标计算模式配置
        self.coordinate_mode_cam1_ = CoordinateMode.DEPTH_MODE     # 倾斜相机坐标计算模式（默认使用深度信息）
        self.coordinate_mode_cam2_ = CoordinateMode.FIXED_Z_MODE   # 水平相机坐标计算模式（默认使用固定z坐标）
        self.fixed_z_value_ = 1.19   # 固定z坐标模式下的z值

        # 图像变量
        self.rgb_image_cam1_ = None
        self.depth_image_cam1_ = None
        self.rgb_image_cam2_ = None
        self.depth_image_cam2_ = None
        
        # 机器人真实位置和姿态（用于验证定位）
        self.robot_position_global_frame_true_ = None
        self.robot_orientation_global_frame_true_ = None
        
        # 定位结果
        self.robot_position_global_frame_estimated_ = None
        self.robot_orientation_global_frame_estimated_ = None

        # 检测置信度阈值
        self.confidence_threshold_ = 0.4  # 置信度阈值，低于此值的检测结果将被过滤
        
        # 地标匹配配置 - 精确控制每个具体地标的启用状态
        self.landmark_matching_config_ = {
            # 门柱 - 禁用
            'P_GoalLeft': True,
            'P_GoalRight': True,
            
            # X型交点 - 仅启用X标记
            'X_PenaltySpot': True,
            'X_CenterCircle': True,
            
            # L型交点 - 禁用
            'L_PenaltyLeft': True,
            'L_CornerLeft': True,
            'L_GoalLeft': True,
            'L_PenaltyRight': True,
            'L_CornerRight': True,
            'L_GoalRight': True,
            
            # T型交点 - 禁用
            'T_PenaltyFrontLeft': True,
            'T_GoalFrontLeft': True,
            'T_CenterBackLeft': False,
            'T_PenaltyFrontRight': True,
            'T_GoalFrontRight': True,
            'T_CenterBackRight': False
        }

        # 坐标系：原点在场地中心，X轴指向球门，z轴向上
        self.field_markers_global_frame_ = {
            # L型交点 - 格式: L_区域_位置
            'L_PenaltyLeft': np.array([2.5, 2.5, 0]),      # 禁区左侧，左位置
            'L_CornerLeft': np.array([4.5, 3.0, 0]),       # 场地角落左侧
            'L_GoalLeft': np.array([3.5, 1.5, 0]),         # 球门区左侧，左位置
            'L_PenaltyRight': np.array([2.5, -2.5, 0]),    # 禁区左侧，右位置
            'L_CornerRight': np.array([4.5, -3.0, 0]),     # 场地角落右侧
            'L_GoalRight': np.array([3.5, -1.5, 0]),       # 球门区左侧，右位置
            
            # T型交点 - 格式: T_区域_位置
            'T_PenaltyFrontLeft': np.array([4.5, 2.5, 0]),   # 禁区前方，左位置
            'T_GoalFrontLeft': np.array([4.5, 1.5, 0]),      # 球门区前方，左位置
            'T_CenterBackLeft': np.array([0.0, 3.0, 0]),     # 中线后方，左位置
            'T_PenaltyFrontRight': np.array([4.5, -2.5, 0]), # 禁区前方，右位置
            'T_GoalFrontRight': np.array([4.5, -1.5, 0]),    # 球门区前方，右位置
            'T_CenterBackRight': np.array([0.0, -3.0, 0]),   # 中线后方，右位置
            
            # X型交点 - 格式: X_描述
            'X_PenaltySpot': np.array([3.0, 0.0, 0]),      # 罚球点
            
            # 门柱 - 格式: P_位置（使用门柱中点，高度0.75米）
            'P_GoalLeft': np.array([4.5, 1.4, 0.75]),      # 左门柱中点
            'P_GoalRight': np.array([4.5, -1.4, 0.75])     # 右门柱中点
        }
        
        # 相机检测结果分别存储
        self.camera1_objects_ = []  # 相机1检测到的对象
        self.camera2_objects_ = []  # 相机2（水平相机）检测到的对象
        
        # 定位优化配置
        self.max_landmarks_for_optimization_ = 15  # 最多使用多少个x坐标最小的地标进行优化，可根据需要调整
        self.depth_filter_threshold_ = 10.0  # 深度过滤阈值（米），超过此深度的地标将被过滤掉
        
        # 深度阈值统一管理
        self.min_valid_depth_ = 0.0   # 最小有效深度阈值（米），深度小于等于此值视为无效
        self.max_valid_depth_ = 10.0  # 最大有效深度阈值（米），深度大于此值视为无效或过远
        
        # 窗口名称
        self.window_name_cam1_ = 'Camera1_Detection'
        self.window_name_cam2_detection_ = 'Camera2_Detection'
        self.window_name_cam2_landmarks_ = 'Camera2_Landmarks'
        
        # 推理时间统计变量
        self.inference_count_ = 0           # 推理次数计数器
        self.total_inference_time_ = 0.0    # 总推理时间
        self.average_inference_time_ = 0.0  # 平均推理时间
        
        # ROS订阅者
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback_cam1)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback_cam1)
        rospy.Subscriber("/horizontal_camera/color/image_raw", Image, self.rgb_callback_cam2)
        rospy.Subscriber("/horizontal_camera/depth/image_raw", Image, self.depth_callback_cam2)
        rospy.Subscriber('/g1_12dof_gazebo/position', Point, self.true_position_callback)
        rospy.Subscriber('/g1_12dof_gazebo/orientation', Quaternion, self.true_orientation_callback)
        
        # 发布机器人位姿和检测到的对象
        self.object_pub_ = rospy.Publisher('/detected_objects_in_robot_frame', ObjectArray, queue_size=10)
        
        # 新增发布器 - 统一话题命名格式
        self.robot_pose_2d_pub_ = rospy.Publisher('/global_position_2d_robot', Pose2D, queue_size=10)  # 发布机器人2D位姿(x,y,yaw)
        self.ball_global_pub_ = rospy.Publisher('/global_position_2d_ball', Pose2D, queue_size=10)  # 发布球的2D全局位置(x,y,yaw=0)
        self.goal_global_pub_ = rospy.Publisher('/global_position_2d_goal', Pose2D, queue_size=10)  # 发布球门的2D全局位置(x,y,yaw=0)
        
        # 坐标计算模式配置输出
        mode_name_cam1 = "使用深度信息模式" if self.coordinate_mode_cam1_ == CoordinateMode.DEPTH_MODE else "固定z坐标模式"
        mode_name_cam2 = "使用深度信息模式" if self.coordinate_mode_cam2_ == CoordinateMode.DEPTH_MODE else "固定z坐标模式"
        # 输出配置信息 
        rospy.loginfo("---------------------------------机器人定位节点已启动---------------------------------")
        rospy.loginfo("检测置信度配置: 置信度阈值 %.2f", self.confidence_threshold_)
        rospy.loginfo("俯视相机坐标计算模式: %s,水平相机坐标计算模式: %s", mode_name_cam1,mode_name_cam2)
        
        # 打印当前配置
        enabled_landmarks = [landmark for landmark, enabled in self.landmark_matching_config_.items() if enabled]
        rospy.loginfo("启用的地标匹配: %s", enabled_landmarks)

    def set_confidence_threshold(self, threshold):
        """
        设置检测置信度阈值
        :param threshold: 置信度阈值，范围为0.0-1.0
        """
        if threshold < 0.0 or threshold > 1.0:
            rospy.logwarn("置信度阈值应在0.0-1.0范围内，设置为0.4")
            threshold = 0.4
        
        old_threshold = self.confidence_threshold_
        self.confidence_threshold_ = threshold
        rospy.loginfo("检测置信度阈值从 %.2f 修改为 %.2f", old_threshold, threshold)

    def set_coordinate_calculation_mode(self, mode, camera_type=None):
        """
        设置坐标计算模式
        :param mode: CoordinateMode枚举值，DEPTH_MODE表示使用深度信息模式，FIXED_Z_MODE表示使用固定z坐标模式
        :param camera_type: CameraType枚举值，OVERHEAD_CAMERA表示俯视相机，HORIZONTAL_CAMERA表示水平相机，None表示同时设置两个相机
        """
        if not isinstance(mode, CoordinateMode):
            rospy.logerr("坐标计算模式必须是CoordinateMode枚举类型")
            return
        
        if camera_type is not None and not isinstance(camera_type, CameraType):
            rospy.logerr("相机类型必须是CameraType枚举类型")
            return
        
        mode_name = "使用深度信息模式" if mode == CoordinateMode.DEPTH_MODE else "固定z坐标模式"
        
        if camera_type == CameraType.OVERHEAD_CAMERA:
            # 只设置俯视相机
            old_mode = self.coordinate_mode_cam1_
            self.coordinate_mode_cam1_ = mode
            old_mode_name = "使用深度信息模式" if old_mode == CoordinateMode.DEPTH_MODE else "固定z坐标模式"
            rospy.loginfo("俯视相机坐标计算模式从 %s 修改为 %s", old_mode_name, mode_name)
        elif camera_type == CameraType.HORIZONTAL_CAMERA:
            # 只设置水平相机
            old_mode = self.coordinate_mode_cam2_
            self.coordinate_mode_cam2_ = mode
            old_mode_name = "使用深度信息模式" if old_mode == CoordinateMode.DEPTH_MODE else "固定z坐标模式"
            rospy.loginfo("水平相机坐标计算模式从 %s 修改为 %s", old_mode_name, mode_name)
        else:
            # 同时设置两个相机
            old_mode_cam1 = self.coordinate_mode_cam1_
            old_mode_cam2 = self.coordinate_mode_cam2_
            self.coordinate_mode_cam1_ = mode
            self.coordinate_mode_cam2_ = mode
            old_mode_name_cam1 = "使用深度信息模式" if old_mode_cam1 == CoordinateMode.DEPTH_MODE else "固定z坐标模式"
            old_mode_name_cam2 = "使用深度信息模式" if old_mode_cam2 == CoordinateMode.DEPTH_MODE else "固定z坐标模式"
            rospy.loginfo("两个相机坐标计算模式都修改为 %s", mode_name)
            rospy.loginfo("  俯视相机: %s -> %s", old_mode_name_cam1, mode_name)
            rospy.loginfo("  水平相机: %s -> %s", old_mode_name_cam2, mode_name)

    def set_fixed_z_value(self, z_value):
        """
        设置固定z坐标模式下的z值
        :param z_value: 固定的z坐标值
        """ 
        old_value = self.fixed_z_value_
        self.fixed_z_value_ = z_value
        rospy.loginfo("固定z坐标值从 %.3f 修改为 %.3f", old_value, z_value)

    def rgb_callback_cam1(self, data): 
        """处理来自相机1的RGB图像"""
        try:
            self.rgb_image_cam1_ = self.bridge_.imgmsg_to_cv2(data, "bgr8")
            if self.rgb_image_cam1_.size == 0:
                rospy.logwarn("接收到的RGB图像（相机1）为空")
                self.rgb_image_cam1_ = None
        except CvBridgeError as e:
            rospy.logerr("无法转换来自相机1的RGB图像: %s", e)

    def depth_callback_cam1(self, data):
        """处理来自相机1的深度图像"""
        try:
            self.depth_image_cam1_ = self.bridge_.imgmsg_to_cv2(data, "16UC1")
            if self.depth_image_cam1_.size == 0:
                rospy.logwarn("接收到的深度图像（相机1）为空")
                self.depth_image_cam1_ = None
        except CvBridgeError as e:
            rospy.logerr("无法转换来自相机1的深度图像: %s", e)

    def rgb_callback_cam2(self, data):
        """处理来自相机2的RGB图像"""
        try:
            self.rgb_image_cam2_ = self.bridge_.imgmsg_to_cv2(data, "bgr8")
            if self.rgb_image_cam2_.size == 0:
                rospy.logwarn("接收到的RGB图像（相机2）为空")
                self.rgb_image_cam2_ = None
        except CvBridgeError as e:
            rospy.logerr("无法转换来自相机2的RGB图像: %s", e)

    def depth_callback_cam2(self, data):
        """处理来自相机2的深度图像"""
        try:
            self.depth_image_cam2_ = self.bridge_.imgmsg_to_cv2(data, "16UC1")
            if self.depth_image_cam2_.size == 0:
                rospy.logwarn("接收到的深度图像（相机2）为空")
                self.depth_image_cam2_ = None
        except CvBridgeError as e:
            rospy.logerr("无法转换来自相机2的深度图像: %s", e)

    def true_position_callback(self, position):
        """处理机器人真实位置信息"""
        self.robot_position_global_frame_true_ = position
        
    def true_orientation_callback(self, orientation):
        """处理机器人真实姿态信息"""
        self.robot_orientation_global_frame_true_ = orientation

    def convert_to_robot_frame(self, x_camera_frame, y_camera_frame, z_camera_frame, camera_offset_robot_frame, camera_orientation):
        """将目标位置从相机坐标系转换为机器人坐标系"""
        
        # 判断是哪个相机（通过相机偏移量来区分）
        is_overhead_camera = np.array_equal(camera_offset_robot_frame, self.camera_offset_cam1_robot_frame_)
        
        if is_overhead_camera:
            # 俯视相机：根据基础的偏航角设置旋转矩阵
            # 通过欧拉角获得旋转矩阵R
            R = euler_matrix(camera_orientation[0], camera_orientation[1], camera_orientation[2], axes='sxyz')[:3, :3]
            rospy.logdebug("俯视相机：使用欧拉角旋转矩阵，角度: [%.3f, %.3f, %.3f]", 
                          camera_orientation[0], camera_orientation[1], camera_orientation[2])
        else:
            # 水平相机：保持不变，使用单位矩阵
            R = np.eye(3)
            rospy.logdebug("水平相机：使用单位矩阵")
        
        T = camera_offset_robot_frame
        return R.dot(np.array([x_camera_frame, y_camera_frame, z_camera_frame])) + T

    def convert_global_to_robot_frame(self, position_global_frame, robot_position_global_frame_true, robot_orientation_global_frame_true):
        """将场地坐标系下的位置转换为机器人坐标系下的位置"""
        if robot_position_global_frame_true is None or robot_orientation_global_frame_true is None:
            return None
        
        # 获取机器人的真实位置和姿态
        robot_pos_global_frame = np.array([robot_position_global_frame_true.x, robot_position_global_frame_true.y, robot_position_global_frame_true.z])
        
        # 将四元数转换为旋转矩阵
        from scipy.spatial.transform import Rotation
        robot_quat = [robot_orientation_global_frame_true.x, robot_orientation_global_frame_true.y, 
                     robot_orientation_global_frame_true.z, robot_orientation_global_frame_true.w]
        R_global_to_robot = Rotation.from_quat(robot_quat).as_matrix().T  # 转置得到逆变换
        
        # 将场地坐标转换为机器人坐标系
        relative_pos_global_frame = position_global_frame - robot_pos_global_frame
        position_robot_frame = R_global_to_robot.dot(relative_pos_global_frame)
        
        return position_robot_frame

    def convert_robot_to_camera_frame(self, position_robot_frame, camera_offset_robot_frame, camera_orientation):
        """将机器人坐标系下的位置转换为相机坐标系下的位置"""
        # # 通过欧拉角获得旋转矩阵R1的逆
        # R1 = euler_matrix(camera_orientation[0], camera_orientation[1], camera_orientation[2], axes='sxyz')[:3, :3]
        # # 定义R2的逆
        # R2_inv = np.array([[0, -1, 0],
        #                   [0, 0, -1],
        #                   [1, 0, 0]])
        # # 总旋转矩阵的逆
        # R_inv = R2_inv.dot(R1.T)
        R_inv = np.eye(3)
        T = camera_offset_robot_frame
        
        # 从机器人坐标系转换为相机坐标系
        relative_pos_robot_frame = position_robot_frame - T
        position_camera_frame = R_inv.dot(relative_pos_robot_frame)
        
        return position_camera_frame

    def convert_robot_frame_to_global_frame(self, position_robot_frame, robot_position_global_frame, robot_orientation_global_frame):
        """将机器人坐标系下的位置转换为全局坐标系下的位置"""
        if robot_position_global_frame is None or robot_orientation_global_frame is None:
            return None
        
        # 获取机器人的全局位置和姿态
        robot_pos_global = np.array([robot_position_global_frame[0], robot_position_global_frame[1], robot_position_global_frame[2]])
        
        # 将旋转矩阵转换为全局到机器人的变换
        R_robot_to_global = robot_orientation_global_frame
        
        # 将机器人坐标系下的点转换到全局坐标系
        position_global_frame = R_robot_to_global.dot(position_robot_frame) + robot_pos_global
        
        return position_global_frame

    def publish_robot_pose_2d(self):
        """发布机器人2D位姿(x,y,yaw)"""
        if self.robot_position_global_frame_estimated_ is not None and self.robot_orientation_global_frame_estimated_ is not None:
            pose_2d_msg = Pose2D()
            pose_2d_msg.x = self.robot_position_global_frame_estimated_[0]
            pose_2d_msg.y = self.robot_position_global_frame_estimated_[1]
            
            # 计算yaw角
            from scipy.spatial.transform import Rotation
            euler_angles = Rotation.from_matrix(self.robot_orientation_global_frame_estimated_).as_euler('xyz', degrees=False)
            pose_2d_msg.theta = euler_angles[2]  # yaw角，弧度
            
            self.robot_pose_2d_pub_.publish(pose_2d_msg)
            rospy.logdebug("发布机器人2D位姿: (%.3f, %.3f, %.3f)", pose_2d_msg.x, pose_2d_msg.y, pose_2d_msg.theta)

    def publish_ball_and_goal_global_positions(self):
        """发布球和球门的全局位置（2D位置，只包含x、y坐标）"""
        if self.robot_position_global_frame_estimated_ is None or self.robot_orientation_global_frame_estimated_ is None:
            return
        
        # 1. 处理球的位置（使用俯视相机camera1的数据）
        ball_objects_cam1 = [obj for obj in self.camera1_objects_ if obj.type == 'Ball']
        if ball_objects_cam1:
            # 取第一个球对象（如果有多个）
            ball_obj = ball_objects_cam1[0]
            ball_robot_frame = np.array([ball_obj.x_robot_frame, ball_obj.y_robot_frame, ball_obj.z_robot_frame])
            
            # 转换到全局坐标系
            ball_global_position = self.convert_robot_frame_to_global_frame(
                ball_robot_frame, 
                self.robot_position_global_frame_estimated_, 
                self.robot_orientation_global_frame_estimated_
            )
            
            if ball_global_position is not None:
                ball_msg = Pose2D()
                ball_msg.x = ball_global_position[0]  # 只使用x坐标
                ball_msg.y = ball_global_position[1]  # 只使用y坐标
                ball_msg.theta = 0.0  # yaw=0，表示2D位置
                self.ball_global_pub_.publish(ball_msg)
                rospy.loginfo("发布球2D全局位置: (%.3f, %.3f)", ball_msg.x, ball_msg.y)
        
        # 2. 处理球门的位置（使用水平相机camera2的数据）
        post_objects_cam2 = [obj for obj in self.camera2_objects_ if obj.type == 'Post']
        if len(post_objects_cam2) >= 2:
            # 如果检测到两个门柱，计算中点
            post1 = post_objects_cam2[0]
            post2 = post_objects_cam2[1]
            
            # 计算门柱中点在机器人坐标系下的位置
            goal_center_robot_frame = np.array([
                (post1.x_robot_frame + post2.x_robot_frame) / 2.0,
                (post1.y_robot_frame + post2.y_robot_frame) / 2.0,
                (post1.z_robot_frame + post2.z_robot_frame) / 2.0
            ])
            
            # 转换到全局坐标系
            goal_global_position = self.convert_robot_frame_to_global_frame(
                goal_center_robot_frame,
                self.robot_position_global_frame_estimated_,
                self.robot_orientation_global_frame_estimated_
            )
            
            if goal_global_position is not None:
                goal_msg = Pose2D()
                goal_msg.x = goal_global_position[0]  # 只使用x坐标
                goal_msg.y = goal_global_position[1]  # 只使用y坐标
                goal_msg.theta = 0.0  # yaw=0，表示2D位置
                self.goal_global_pub_.publish(goal_msg)
                rospy.loginfo("发布球门2D全局位置: (%.3f, %.3f)", goal_msg.x, goal_msg.y)
                
        elif len(post_objects_cam2) == 1:
            # 如果只检测到一个门柱，直接使用该门柱位置
            post_obj = post_objects_cam2[0]
            post_robot_frame = np.array([post_obj.x_robot_frame, post_obj.y_robot_frame, post_obj.z_robot_frame])
            
            # 转换到全局坐标系
            post_global_position = self.convert_robot_frame_to_global_frame(
                post_robot_frame,
                self.robot_position_global_frame_estimated_,
                self.robot_orientation_global_frame_estimated_
            )
            
            if post_global_position is not None:
                goal_msg = Pose2D()
                goal_msg.x = post_global_position[0]  # 只使用x坐标
                goal_msg.y = post_global_position[1]  # 只使用y坐标
                goal_msg.theta = 0.0  # yaw=0，表示2D位置
                self.goal_global_pub_.publish(goal_msg)
                rospy.loginfo("发布单门柱2D全局位置: (%.3f, %.3f)", goal_msg.x, goal_msg.y)

    def calculate_true_values_for_objects(self, objects, camera_offset_robot_frame, camera_orientation):
        """为已匹配标签的对象计算真值坐标"""
        if self.robot_position_global_frame_true_ is None or self.robot_orientation_global_frame_true_ is None:
            return
        
        for obj in objects:
            if obj.label and obj.label in self.field_markers_global_frame_:
                # 获取场地坐标系下的真实位置
                position_global_frame = self.field_markers_global_frame_[obj.label]
                
                # 转换为机器人坐标系下的真值
                position_robot_frame_true = self.convert_global_to_robot_frame(
                    position_global_frame, self.robot_position_global_frame_true_, self.robot_orientation_global_frame_true_)
                
                if position_robot_frame_true is not None:
                    obj.x_robot_frame_true = position_robot_frame_true[0]
                    obj.y_robot_frame_true = position_robot_frame_true[1]
                    obj.z_robot_frame_true = position_robot_frame_true[2]
                    
                    # 转换为相机坐标系下的真值
                    position_camera_frame_true = self.convert_robot_to_camera_frame(
                        position_robot_frame_true, camera_offset_robot_frame, camera_orientation)
                    
                    obj.x_camera_frame_true = position_camera_frame_true[0]
                    obj.y_camera_frame_true = position_camera_frame_true[1]
                    obj.z_camera_frame_true = position_camera_frame_true[2]
                    
                    obj.has_true_values = True

    def log_object_details(self, objects, camera_name):
        """详细记录检测对象的信息"""
        if not objects:
            return
            
        rospy.loginfo("=============== %s 检测详情 ===============", camera_name)
        
        for i, obj in enumerate(objects):
            rospy.loginfo("--- 对象 %d ---", i + 1)
            rospy.loginfo("类型: %s", obj.type)
            
            # 匹配的标签
            if obj.label:
                rospy.loginfo("匹配标签: %s", obj.label)
            
            # 相机坐标系坐标
            rospy.loginfo("相机系坐标: (%.3f, %.3f, %.3f)", obj.x_camera_frame, obj.y_camera_frame, obj.z_camera_frame)
            
            # 机器人坐标系坐标
            rospy.loginfo("机器人系坐标: (%.3f, %.3f, %.3f)", obj.x_robot_frame, obj.y_robot_frame, obj.z_robot_frame)
            
            # 如果有真值，显示真值和误差
            if obj.has_true_values:
                rospy.loginfo("相机系真值: (%.3f, %.3f, %.3f)", 
                             obj.x_camera_frame_true, obj.y_camera_frame_true, obj.z_camera_frame_true)
                rospy.loginfo("机器人系真值: (%.3f, %.3f, %.3f)", 
                             obj.x_robot_frame_true, obj.y_robot_frame_true, obj.z_robot_frame_true)
                # 计算误差
                robot_error = np.linalg.norm([obj.x_robot_frame - obj.x_robot_frame_true, 
                                            obj.y_robot_frame - obj.y_robot_frame_true, 
                                            obj.z_robot_frame - obj.z_robot_frame_true])
                cam_error = np.linalg.norm([obj.x_camera_frame - obj.x_camera_frame_true, 
                                          obj.y_camera_frame - obj.y_camera_frame_true, 
                                          obj.z_camera_frame - obj.z_camera_frame_true])
                
                rospy.loginfo("相机系误差: %.3f m", cam_error)
                rospy.loginfo("机器人系误差: %.3f m", robot_error)
            
            rospy.loginfo("置信度: %.3f", obj.confidence)
            rospy.loginfo("像素坐标: (%d, %d)", obj.pixel_x, obj.pixel_y)
        
        rospy.loginfo("===============================================")

    def process_detection(self, rgb_image, depth_image, camera_offset_robot_frame, camera_orientation, target_classes):
        """处理目标检测，返回检测到的Object列表"""
        detected_objects = []
        
        start_time = time.time()
        results = self.model_.track(source=rgb_image, save=False, verbose=False)
        current_inference_time = time.time() - start_time
        
        # 更新统计信息
        self.inference_count_ += 1
        self.total_inference_time_ += current_inference_time
        self.average_inference_time_ = self.total_inference_time_ / self.inference_count_
        
        # 打印推理时间统计信息
        print(f"YOLO推理时间: {current_inference_time:.4f}s | 平均推理时间: {self.average_inference_time_:.4f}s | 推理次数: {self.inference_count_}")
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls)
                if self.model_.names[cls] in target_classes:
                    # 检查置信度是否满足阈值要求
                    confidence = float(box.conf)
                    if confidence < self.confidence_threshold_:
                        rospy.logdebug("跳过低置信度检测: 类型=%s, 置信度=%.3f (阈值=%.2f)", 
                                       self.model_.names[cls], confidence, self.confidence_threshold_)
                        continue
                    
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # 根据目标类型选择不同的坐标点
                    if self.model_.names[cls] == 'Post':
                        # 门柱使用下边界坐标
                        center_x = (x1 + x2) // 2  # 水平中心
                        center_y = y2  # 垂直下边界
                        rospy.logdebug("门柱检测使用下边界坐标: (%d, %d)", center_x, center_y)
                    else:
                        # 其他目标使用中心点
                        center_x = (x1 + x2) // 2  # 水平中心
                        center_y = (y1 + y2) // 2  # 垂直中心
                    
                    # 检查像素坐标是否在图像边界内
                    img_height, img_width = depth_image.shape
                    if center_x < 0 or center_x >= img_width or center_y < 0 or center_y >= img_height:
                        rospy.logdebug("跳过越界像素坐标: (%d, %d), 图像尺寸: (%d, %d)", 
                                       center_x, center_y, img_width, img_height)
                        continue
                    
                    # 如果坐标在边界上，调整到安全范围内
                    center_x = max(0, min(center_x, img_width - 1))
                    center_y = max(0, min(center_y, img_height - 1))
                    
                    # 根据相机类型选择对应的坐标计算模式
                    is_camera1 = np.array_equal(camera_offset_robot_frame, self.camera_offset_cam1_robot_frame_)
                    current_mode = self.coordinate_mode_cam1_ if is_camera1 else self.coordinate_mode_cam2_
                    camera_name = "俯视相机" if is_camera1 else "水平相机"
                    
                    # 根据坐标计算模式选择不同的计算方法
                    if current_mode == CoordinateMode.DEPTH_MODE:
                        # 模式1: 使用深度信息模式
                        depth = depth_image[center_y, center_x] / 1000.0
                        if depth <= self.min_valid_depth_ or depth > self.max_valid_depth_:  # 跳过无效深度或过远的点
                            continue
                        
                        # 使用实际深度值计算相机坐标系中的3D坐标
                        x_camera_frame = depth
                        y_camera_frame = -(center_x - self.cx_) * depth / self.fx_  
                        z_camera_frame = -(center_y - self.cy_) * depth / self.fy_
                        
                        rospy.logdebug("%s深度模式 - 深度: %.3f, 相机坐标: (%.3f, %.3f, %.3f)", 
                                       camera_name, depth, x_camera_frame, y_camera_frame, z_camera_frame)
                    else:
                        # 模式2: 固定z坐标模式
                        # 检查center_y是否等于cy，避免除零错误
                        if abs(center_y - self.cy_) < 1e-6:
                            rospy.logdebug("%s跳过center_y接近cy的像素点，避免除零错误: center_y=%d, cy=%.1f", 
                                           camera_name, center_y, self.cy_)
                            continue
                        
                        # 固定z坐标，反推深度值
                        z_camera_frame = self.fixed_z_value_
                        depth = -z_camera_frame * self.fy_ / (center_y - self.cy_)
                        
                        # 检查反推的深度值是否合理
                        if depth <= self.min_valid_depth_ or depth > self.max_valid_depth_:
                            rospy.logdebug("%s跳过不合理的反推深度值: %.3f 在位置 (%d, %d)", camera_name, depth, center_x, center_y)
                            continue
                        
                        # 使用反推的深度值计算x和y坐标 
                        x_camera_frame = depth
                        y_camera_frame = -(center_x - self.cx_) * depth / self.fx_
                        
                        rospy.logdebug("%s固定z模式 - 固定z: %.3f, 反推深度: %.3f, 相机坐标: (%.3f, %.3f, %.3f)", 
                                       camera_name, self.fixed_z_value_, depth, x_camera_frame, y_camera_frame, z_camera_frame)
                    
                    # 转换到机器人坐标系
                    position_robot_frame = self.convert_to_robot_frame(x_camera_frame, y_camera_frame, z_camera_frame, camera_offset_robot_frame, camera_orientation)
                    
                    obj = ExtendedObject()
                    obj.type = self.model_.names[cls]
                    # 保存机器人坐标系坐标（用于最终优化）
                    obj.x_robot_frame = position_robot_frame[0]
                    obj.y_robot_frame = position_robot_frame[1]
                    obj.z_robot_frame = position_robot_frame[2]
                    
                    # 同时保存相机坐标系坐标（用于匹配）和相机信息
                    obj.x_camera_frame = x_camera_frame
                    obj.y_camera_frame = y_camera_frame
                    obj.z_camera_frame = z_camera_frame
                    obj.camera_id = 1 if np.array_equal(camera_offset_robot_frame, self.camera_offset_cam1_robot_frame_) else 2
                    
                    # 保存RGB图像像素信息
                    obj.pixel_x = center_x
                    obj.pixel_y = center_y
                    obj.bbox_x1 = x1
                    obj.bbox_y1 = y1
                    obj.bbox_x2 = x2
                    obj.bbox_y2 = y2
                    obj.confidence = confidence
                    
                    # 添加到检测结果列表
                    detected_objects.append(obj)
                    rospy.logdebug("检测通过: 类型=%s, 置信度=%.3f, 位置=(%d,%d)", 
                                   obj.type, confidence, center_x, center_y)
        
        return detected_objects

    def draw_axes(self, image, origin, axis_length=50, thickness=2):
        """
        在图像上绘制坐标轴
        :param image: 输入图像
        :param origin: 坐标原点 (x, y)
        :param axis_length: 坐标轴长度
        :param thickness: 线宽
        """
        # X轴（红色）
        cv2.arrowedLine(image, origin, (origin[0] + axis_length, origin[1]), (0, 0, 255), thickness, tipLength=0.2)
        cv2.putText(image, 'X', (origin[0] + axis_length + 5, origin[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        # Y轴（绿色）
        cv2.arrowedLine(image, origin, (origin[0], origin[1] + axis_length), (0, 255, 0), thickness, tipLength=0.2)
        cv2.putText(image, 'Y', (origin[0], origin[1] + axis_length + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # 原点（蓝色圆点）
        cv2.circle(image, origin, 5, (255, 0, 0), -1)

    def estimate_pose(self, P_global_frame, P_robot_frame):
        """使用最小二乘法估计机器人2D位姿（仅优化x、y和yaw）"""
        
        # 特殊处理单个地标点的情况
        if len(P_global_frame) == 1:
            rospy.loginfo("单地标定位: 只估计位置，假设yaw=0")
            
            # 使用单个地标点估计位置，假设yaw=0
            global_point = P_global_frame[0][:2]  # 只使用x, y坐标
            robot_point = P_robot_frame[0][:2]    # 只使用x, y坐标
            
            # 假设yaw=0，直接计算位置
            # global_point = robot_position + robot_point (当yaw=0时)
            # 所以 robot_position = global_point - robot_point
            estimated_position = global_point - robot_point
            
            # 构造3D旋转矩阵（yaw=0）
            R = np.array([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
            
            # 构造3D平移向量
            T = np.array([estimated_position[0], estimated_position[1], 0.0])
            
            rospy.loginfo("单地标定位结果: 位置(%.3f, %.3f), yaw=0°", T[0], T[1])
            
            return R, T
        
        # 原有的多地标最小二乘法
        def residuals(params):
            # params = [x, y, yaw] - 只优化2D位置和偏航角
            x, y, yaw = params
            
            # 构造2D旋转矩阵（绕z轴旋转）
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            R_2d = np.array([[cos_yaw, -sin_yaw],
                            [sin_yaw,  cos_yaw]])
            
            # 平移向量（只考虑x、y）
            T_2d = np.array([x, y])
            
            # 只使用x、y坐标进行匹配，忽略z坐标
            P_global_frame_2d = P_global_frame[:, :2]  # 取前两列（x、y）
            P_robot_frame_2d = P_robot_frame[:, :2]  # 取前两列（x、y）
            
            # 计算误差
            error = P_global_frame_2d - (np.dot(R_2d, P_robot_frame_2d.T).T + T_2d)
            return error.flatten()
        
        # 初始参数估计 [x, y, yaw]
        initial_params = np.array([0.0, 0.0, 0.0])
        result = least_squares(residuals, initial_params)
        
        if result.success:
            x, y, yaw = result.x
            
            # 构造完整的3D旋转矩阵（只有yaw角，roll=pitch=0）
            R = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw),  np.cos(yaw), 0],
                         [0,            0,           1]])
            
            # 构造完整的3D平移向量（z=0）
            T = np.array([x, y, 0.0])
            
            rospy.loginfo("多地标定位成功: 位置(%.3f, %.3f), 偏航角 %.1f°", 
                         x, y, np.degrees(yaw))
            
            return R, T
        else:
            rospy.logerr("多地标位姿估计失败")
            return None, None
        return pose

    def match_landmarks(self, detected_objects_3d):
        """
        根据检测到的3D物体匹配对应的地标标签，直接在ExtendedObject中设置label属性
        输入: detected_objects_3d - 检测到的3D物体列表（只使用水平相机的检测结果）
        返回: landmarks_matched - 字典，键为地标标签，值为机器人坐标系3D位置
        """
        detected_objects = []
        for obj in detected_objects_3d:
            # 直接使用ExtendedObject中存储的像素坐标
            detected_objects.append({
                'type': obj.type,
                'pixel_x': obj.pixel_x,
                'pixel_y': obj.pixel_y,
                'obj_ref': obj  # 保存原始对象引用，用于最后设置标签
            })
        
        landmarks_2d = {}
        
        # 分类检测到的物体
        posts = [obj for obj in detected_objects if obj['type'] == 'Post']
        x_marks = [obj for obj in detected_objects if obj['type'] == 'X']
        l_marks = [obj for obj in detected_objects if obj['type'] == 'L']
        t_marks = [obj for obj in detected_objects if obj['type'] == 'T']
        
        rospy.logdebug("地标匹配开始: Posts=%d, X=%d, L=%d, T=%d", 
                       len(posts), len(x_marks), len(l_marks), len(t_marks))
        
        # **处理门柱**
        if posts and (self.landmark_matching_config_.get('P_GoalLeft', False) or 
                      self.landmark_matching_config_.get('P_GoalRight', False)):
            if len(posts) >= 2:
                # 根据x坐标排序，左门柱x值小，右门柱x值大
                posts_sorted = sorted(posts, key=lambda p: p['pixel_x'])
                if self.landmark_matching_config_.get('P_GoalLeft', False):
                    landmarks_2d['P_GoalLeft'] = (posts_sorted[0]['pixel_x'], posts_sorted[0]['pixel_y'])
                if self.landmark_matching_config_.get('P_GoalRight', False):
                    landmarks_2d['P_GoalRight'] = (posts_sorted[-1]['pixel_x'], posts_sorted[-1]['pixel_y'])
                rospy.logdebug("门柱匹配: 左门柱(%d,%d), 右门柱(%d,%d)", 
                               posts_sorted[0]['pixel_x'], posts_sorted[0]['pixel_y'],
                               posts_sorted[-1]['pixel_x'], posts_sorted[-1]['pixel_y'])
            elif len(posts) == 1:
                single_post = posts[0]
                post_x = single_post['pixel_x']
                
                # 检查门柱右侧（x增大方向）是否有L型或T型标记
                right_side_markers = []
                for marker in l_marks + t_marks:
                    if marker['pixel_x'] > post_x:
                        right_side_markers.append(marker)
                
                # 检查门柱左侧（x减小方向）是否有L型或T型标记
                left_side_markers = []
                for marker in l_marks + t_marks:
                    if marker['pixel_x'] < post_x:
                        left_side_markers.append(marker)
                
                # 根据标记分布判断门柱类型
                if len(right_side_markers) > len(left_side_markers):
                    # 右侧标记更多，说明是右门柱
                    if self.landmark_matching_config_.get('P_GoalRight', False):
                        landmarks_2d['P_GoalRight'] = (single_post['pixel_x'], single_post['pixel_y'])
                        rospy.logdebug("根据右侧标记判断为右门柱: (%d,%d), 右侧标记数=%d, 左侧标记数=%d", 
                                       single_post['pixel_x'], single_post['pixel_y'],
                                       len(right_side_markers), len(left_side_markers))
                elif len(left_side_markers) > len(right_side_markers):
                    # 左侧标记更多，说明是左门柱
                    if self.landmark_matching_config_.get('P_GoalLeft', False):
                        landmarks_2d['P_GoalLeft'] = (single_post['pixel_x'], single_post['pixel_y'])
                        rospy.logdebug("根据左侧标记判断为左门柱: (%d,%d), 左侧标记数=%d, 右侧标记数=%d", 
                                       single_post['pixel_x'], single_post['pixel_y'],
                                       len(left_side_markers), len(right_side_markers))
                else:
                    # 如果两侧标记数量相等或都没有标记，则无法确定，记录警告
                    rospy.logwarn("无法根据周围标记确定门柱类型: 左侧标记数=%d, 右侧标记数=%d", 
                                  len(left_side_markers), len(right_side_markers))
                    # 可以选择不分配，或者作为fallback使用原来的cx判断
                    if single_post['pixel_x'] < self.cx_:  # fallback到原来的逻辑
                        if self.landmark_matching_config_.get('P_GoalLeft', False):
                            landmarks_2d['P_GoalLeft'] = (single_post['pixel_x'], single_post['pixel_y'])
                            rospy.logdebug("fallback判断为左门柱: (%d,%d)", 
                                           single_post['pixel_x'], single_post['pixel_y'])
                    else:
                        if self.landmark_matching_config_.get('P_GoalRight', False):
                            landmarks_2d['P_GoalRight'] = (single_post['pixel_x'], single_post['pixel_y'])
                            rospy.logdebug("fallback判断为右门柱: (%d,%d)", 
                                           single_post['pixel_x'], single_post['pixel_y'])
        
        # **处理X型交点**
        if x_marks and (self.landmark_matching_config_.get('X_PenaltySpot', False) or 
                        self.landmark_matching_config_.get('X_CenterCircle', False)):
            # 按y坐标排序，y值小的是罚球点X，y值大的是中圈X
            x_sorted_by_y = sorted(x_marks, key=lambda x: x['pixel_y'])
            
            if len(x_marks) >= 2:
                if self.landmark_matching_config_.get('X_PenaltySpot', False):
                    penalty_x = x_sorted_by_y[0]  # y值最小（最靠上）
                    landmarks_2d['X_PenaltySpot'] = (penalty_x['pixel_x'], penalty_x['pixel_y'])
                    rospy.logdebug("罚球点X匹配: (%d,%d)", penalty_x['pixel_x'], penalty_x['pixel_y'])
                if self.landmark_matching_config_.get('X_CenterCircle', False):
                    center_x = x_sorted_by_y[-1]  # y值最大（最靠下）
                    landmarks_2d['X_CenterCircle'] = (center_x['pixel_x'], center_x['pixel_y'])
                    rospy.logdebug("中圈X匹配: (%d,%d)", center_x['pixel_x'], center_x['pixel_y'])
            elif len(x_marks) == 1:
                single_x = x_marks[0]
                # 单个X标记时，优先分配给罚球点
                if self.landmark_matching_config_.get('X_PenaltySpot', False):
                    landmarks_2d['X_PenaltySpot'] = (single_x['pixel_x'], single_x['pixel_y'])
                    rospy.logdebug("单一X为罚球点: (%d,%d)", single_x['pixel_x'], single_x['pixel_y'])
                elif self.landmark_matching_config_.get('X_CenterCircle', False):
                    landmarks_2d['X_CenterCircle'] = (single_x['pixel_x'], single_x['pixel_y'])
                    rospy.logdebug("单一X为中圈: (%d,%d)", single_x['pixel_x'], single_x['pixel_y'])
        
        # **处理L型交点**
        if l_marks and any(self.landmark_matching_config_.get(key, False) for key in 
                          ['L_PenaltyLeft', 'L_CornerLeft', 'L_GoalLeft', 'L_PenaltyRight', 'L_CornerRight', 'L_GoalRight']):
            
            # 智能选择分组参考点
            reference_x = None
            grouping_strategy = None
            
            # 优先使用X标记作为参考点
            if 'X_PenaltySpot' in landmarks_2d:
                reference_x = landmarks_2d['X_PenaltySpot'][0]
                grouping_strategy = "X_PenaltySpot"
                rospy.logdebug("使用罚球点X标记作为参考: x=%d", reference_x)
            elif 'X_CenterCircle' in landmarks_2d:
                reference_x = landmarks_2d['X_CenterCircle'][0]
                grouping_strategy = "X_CenterCircle"
                rospy.logdebug("使用中圈X标记作为参考: x=%d", reference_x)
            # 没有X标记时，使用球门柱信息
            elif 'P_GoalLeft' in landmarks_2d and 'P_GoalRight' in landmarks_2d:
                # 有两个球门柱，使用中点x坐标作为参考
                left_post_x = landmarks_2d['P_GoalLeft'][0]
                right_post_x = landmarks_2d['P_GoalRight'][0]
                reference_x = (left_post_x + right_post_x) // 2
                grouping_strategy = "both_posts"
                rospy.logdebug("使用两个门柱中点作为参考: 左门柱x=%d, 右门柱x=%d, 中点x=%d", 
                               left_post_x, right_post_x, reference_x)
            elif 'P_GoalLeft' in landmarks_2d:
                # 只有左门柱，所有L标记都归为左侧
                grouping_strategy = "left_post_only"
                rospy.logdebug("只有左门柱，所有L标记归为左侧")
            elif 'P_GoalRight' in landmarks_2d:
                # 只有右门柱，所有L标记都归为右侧
                grouping_strategy = "right_post_only"
                rospy.logdebug("只有右门柱，所有L标记归为右侧")
            else:
                # 没有任何参考标记，使用图像中心作为fallback
                reference_x = self.cx_
                grouping_strategy = "image_center"
                rospy.logdebug("未检测到参考标记，使用图像中心作为参考: x=%d", reference_x)
            
            # 根据分组策略处理L型交点
            if grouping_strategy == "left_post_only":
                # 只有左门柱，所有L标记都按左侧逻辑处理
                rospy.logdebug("左侧门柱策略: 将所有%d个L标记按左侧逻辑处理", len(l_marks))
                if len(l_marks) == 3:
                    # 按x坐标排序，x最大的是L_GoalLeft
                    l_sorted_by_x = sorted(l_marks, key=lambda l: l['pixel_x'], reverse=True)
                    if self.landmark_matching_config_.get('L_GoalLeft', False):
                        landmarks_2d['L_GoalLeft'] = (l_sorted_by_x[0]['pixel_x'], l_sorted_by_x[0]['pixel_y'])
                        rospy.logdebug("L_GoalLeft匹配: (%d,%d)", l_sorted_by_x[0]['pixel_x'], l_sorted_by_x[0]['pixel_y'])
                    
                    # 剩下的根据y来区分
                    remaining_l = l_sorted_by_x[1:]
                    remaining_l_by_y = sorted(remaining_l, key=lambda l: l['pixel_y'])
                    # y小的是L_CornerLeft
                    if self.landmark_matching_config_.get('L_CornerLeft', False):
                        landmarks_2d['L_CornerLeft'] = (remaining_l_by_y[0]['pixel_x'], remaining_l_by_y[0]['pixel_y'])
                        rospy.logdebug("L_CornerLeft匹配: (%d,%d)", remaining_l_by_y[0]['pixel_x'], remaining_l_by_y[0]['pixel_y'])
                    # y大的是L_PenaltyLeft
                    if self.landmark_matching_config_.get('L_PenaltyLeft', False):
                        landmarks_2d['L_PenaltyLeft'] = (remaining_l_by_y[1]['pixel_x'], remaining_l_by_y[1]['pixel_y'])
                        rospy.logdebug("L_PenaltyLeft匹配: (%d,%d)", remaining_l_by_y[1]['pixel_x'], remaining_l_by_y[1]['pixel_y'])
                        
            elif grouping_strategy == "right_post_only":
                # 只有右门柱，所有L标记都按右侧逻辑处理
                rospy.logdebug("右侧门柱策略: 将所有%d个L标记按右侧逻辑处理", len(l_marks))
                if len(l_marks) == 3:
                    # 按x坐标排序，x最小的是L_GoalRight
                    l_sorted_by_x = sorted(l_marks, key=lambda l: l['pixel_x'])
                    if self.landmark_matching_config_.get('L_GoalRight', False):
                        landmarks_2d['L_GoalRight'] = (l_sorted_by_x[0]['pixel_x'], l_sorted_by_x[0]['pixel_y'])
                        rospy.logdebug("L_GoalRight匹配: (%d,%d)", l_sorted_by_x[0]['pixel_x'], l_sorted_by_x[0]['pixel_y'])
                    
                    # 剩下的根据y来区分
                    remaining_l = l_sorted_by_x[1:]
                    remaining_l_by_y = sorted(remaining_l, key=lambda l: l['pixel_y'])
                    # y小的是L_CornerRight
                    if self.landmark_matching_config_.get('L_CornerRight', False):
                        landmarks_2d['L_CornerRight'] = (remaining_l_by_y[0]['pixel_x'], remaining_l_by_y[0]['pixel_y'])
                        rospy.logdebug("L_CornerRight匹配: (%d,%d)", remaining_l_by_y[0]['pixel_x'], remaining_l_by_y[0]['pixel_y'])
                    # y大的是L_PenaltyRight
                    if self.landmark_matching_config_.get('L_PenaltyRight', False):
                        landmarks_2d['L_PenaltyRight'] = (remaining_l_by_y[1]['pixel_x'], remaining_l_by_y[1]['pixel_y'])
                        rospy.logdebug("L_PenaltyRight匹配: (%d,%d)", remaining_l_by_y[1]['pixel_x'], remaining_l_by_y[1]['pixel_y'])      
            else:
                # 使用参考点进行左右分组（原有逻辑）
                left_l = [l for l in l_marks if l['pixel_x'] < reference_x]
                right_l = [l for l in l_marks if l['pixel_x'] >= reference_x]
                
                rospy.logdebug("L型交点分组: 左侧%d个，右侧%d个 (参考点x=%d, 策略=%s)", 
                               len(left_l), len(right_l), reference_x, grouping_strategy)
                
                # 右侧有3个L标记时，启动右侧逻辑
                if len(right_l) == 3:
                    rospy.logdebug("右侧有3个L标记，使用右侧匹配逻辑")
                    
                    # 处理右侧L型交点
                    # 按x坐标排序，x最小的是L_GoalRight
                    right_sorted_by_x = sorted(right_l, key=lambda l: l['pixel_x'])
                    if 'L_GoalRight' not in landmarks_2d and \
                       self.landmark_matching_config_.get('L_GoalRight', False):
                        landmarks_2d['L_GoalRight'] = (right_sorted_by_x[0]['pixel_x'], right_sorted_by_x[0]['pixel_y'])
                        rospy.logdebug("L_GoalRight匹配: (%d,%d)", right_sorted_by_x[0]['pixel_x'], right_sorted_by_x[0]['pixel_y'])
                    
                    # 剩下的根据y来区分
                    remaining_right = right_sorted_by_x[1:]
                    remaining_right_by_y = sorted(remaining_right, key=lambda l: l['pixel_y'])
                    # y小的是L_CornerRight
                    if 'L_CornerRight' not in landmarks_2d and \
                       self.landmark_matching_config_.get('L_CornerRight', False):
                        landmarks_2d['L_CornerRight'] = (remaining_right_by_y[0]['pixel_x'], remaining_right_by_y[0]['pixel_y'])
                        rospy.logdebug("L_CornerRight匹配: (%d,%d)", remaining_right_by_y[0]['pixel_x'], remaining_right_by_y[0]['pixel_y'])
                    # y大的是L_PenaltyRight
                    if 'L_PenaltyRight' not in landmarks_2d and \
                       self.landmark_matching_config_.get('L_PenaltyRight', False):
                        landmarks_2d['L_PenaltyRight'] = (remaining_right_by_y[1]['pixel_x'], remaining_right_by_y[1]['pixel_y'])
                        rospy.logdebug("L_PenaltyRight匹配: (%d,%d)", remaining_right_by_y[1]['pixel_x'], remaining_right_by_y[1]['pixel_y'])
                
                # 左侧有3个L标记时，启动左侧逻辑
                if len(left_l) == 3:
                    rospy.logdebug("左侧有3个L标记，使用左侧匹配逻辑")
                    
                    # 处理左侧L型交点（对应的镜像逻辑）
                    # 按x坐标排序，x最大的是L_GoalLeft
                    left_sorted_by_x = sorted(left_l, key=lambda l: l['pixel_x'], reverse=True)
                    if 'L_GoalLeft' not in landmarks_2d and \
                       self.landmark_matching_config_.get('L_GoalLeft', False):
                        landmarks_2d['L_GoalLeft'] = (left_sorted_by_x[0]['pixel_x'], left_sorted_by_x[0]['pixel_y'])
                        rospy.logdebug("L_GoalLeft匹配: (%d,%d)", left_sorted_by_x[0]['pixel_x'], left_sorted_by_x[0]['pixel_y'])
                    
                    # 剩下的根据y来区分
                    remaining_left = left_sorted_by_x[1:]
                    remaining_left_by_y = sorted(remaining_left, key=lambda l: l['pixel_y'])
                    # y小的是L_CornerLeft
                    if 'L_CornerLeft' not in landmarks_2d and \
                       self.landmark_matching_config_.get('L_CornerLeft', False):
                        landmarks_2d['L_CornerLeft'] = (remaining_left_by_y[0]['pixel_x'], remaining_left_by_y[0]['pixel_y'])
                        rospy.logdebug("L_CornerLeft匹配: (%d,%d)", remaining_left_by_y[0]['pixel_x'], remaining_left_by_y[0]['pixel_y'])
                    # y大的是L_PenaltyLeft
                    if 'L_PenaltyLeft' not in landmarks_2d and \
                       self.landmark_matching_config_.get('L_PenaltyLeft', False):
                        landmarks_2d['L_PenaltyLeft'] = (remaining_left_by_y[1]['pixel_x'], remaining_left_by_y[1]['pixel_y'])
                        rospy.logdebug("L_PenaltyLeft匹配: (%d,%d)", remaining_left_by_y[1]['pixel_x'], remaining_left_by_y[1]['pixel_y'])
                
                # 其他情况的处理逻辑
                if len(left_l) != 3 and len(right_l) != 3 and grouping_strategy not in ["left_post_only", "right_post_only"]:
                    rospy.logdebug("左右两侧都不是3个L标记，当前分组: 左侧%d个，右侧%d个", len(left_l), len(right_l))
        
        # **处理T型交点（基于门柱或L标记的距离分配）**
        if t_marks and any(self.landmark_matching_config_.get(key, False) for key in 
                          ['T_PenaltyFrontLeft', 'T_GoalFrontLeft', 'T_CenterBackLeft', 'T_PenaltyFrontRight', 'T_GoalFrontRight', 'T_CenterBackRight']):
            
            # 使用与L型交点相同的智能分组逻辑
            reference_x = None
            grouping_strategy = None
            
            # 优先使用X标记作为参考点
            if 'X_PenaltySpot' in landmarks_2d:
                reference_x = landmarks_2d['X_PenaltySpot'][0]
                grouping_strategy = "X_PenaltySpot"
            elif 'X_CenterCircle' in landmarks_2d:
                reference_x = landmarks_2d['X_CenterCircle'][0]
                grouping_strategy = "X_CenterCircle"
            # 没有X标记时，使用球门柱信息
            elif 'P_GoalLeft' in landmarks_2d and 'P_GoalRight' in landmarks_2d:
                # 有两个球门柱，使用中点x坐标作为参考
                left_post_x = landmarks_2d['P_GoalLeft'][0]
                right_post_x = landmarks_2d['P_GoalRight'][0]
                reference_x = (left_post_x + right_post_x) // 2
                grouping_strategy = "both_posts"
            elif 'P_GoalLeft' in landmarks_2d:
                # 只有左门柱，所有T标记都归为左侧
                grouping_strategy = "left_post_only"
            elif 'P_GoalRight' in landmarks_2d:
                # 只有右门柱，所有T标记都归为右侧
                grouping_strategy = "right_post_only"
            else:
                reference_x = self.cx_
                grouping_strategy = "image_center"
            
            # 根据分组策略处理T型交点
            if grouping_strategy == "left_post_only":
                # 只有左门柱，所有T标记都按左侧逻辑处理
                rospy.logdebug("左侧门柱策略: 将所有%d个T标记按左侧逻辑处理", len(t_marks))
                left_t = t_marks  # 所有T标记都归为左侧
                right_t = []
            elif grouping_strategy == "right_post_only":
                # 只有右门柱，所有T标记都按右侧逻辑处理
                rospy.logdebug("右侧门柱策略: 将所有%d个T标记按右侧逻辑处理", len(t_marks))
                left_t = []
                right_t = t_marks  # 所有T标记都归为右侧
            else:
                # 基于参考点分成左右两个部分
                left_t = [t for t in t_marks if t['pixel_x'] < reference_x]
                right_t = [t for t in t_marks if t['pixel_x'] >= reference_x]
            
            # 处理右侧T型交点
            if len(right_t) >= 2:
                if 'P_GoalRight' in landmarks_2d:
                    # 有右门柱时，基于与门柱的距离分配
                    goal_pos = landmarks_2d['P_GoalRight']
                    right_t_with_dist = []
                    for t in right_t:
                        dist = np.hypot(t['pixel_x'] - goal_pos[0], t['pixel_y'] - goal_pos[1])
                        right_t_with_dist.append((t, dist))
                    right_t_with_dist.sort(key=lambda x: x[1])  # 按距离排序
                    
                    # 离右门柱近的是T_GoalFrontRight
                    if 'T_GoalFrontRight' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_GoalFrontRight', False):
                        closest_t = right_t_with_dist[0][0]
                        landmarks_2d['T_GoalFrontRight'] = (closest_t['pixel_x'], closest_t['pixel_y'])
                        rospy.logdebug("T_GoalFrontRight匹配: (%d,%d)", closest_t['pixel_x'], closest_t['pixel_y'])
                    
                    # 另一个是T_PenaltyFrontRight
                    if len(right_t_with_dist) >= 2 and 'T_PenaltyFrontRight' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_PenaltyFrontRight', False):
                        second_t = right_t_with_dist[1][0]
                        landmarks_2d['T_PenaltyFrontRight'] = (second_t['pixel_x'], second_t['pixel_y'])
                        rospy.logdebug("T_PenaltyFrontRight匹配: (%d,%d)", second_t['pixel_x'], second_t['pixel_y'])
                        
                elif 'L_CornerRight' in landmarks_2d:
                    # 没有门柱但有L_CornerRight时，基于与L_CornerRight的距离分配
                    corner_pos = landmarks_2d['L_CornerRight']
                    right_t_with_dist = []
                    for t in right_t:
                        dist = np.hypot(t['pixel_x'] - corner_pos[0], t['pixel_y'] - corner_pos[1])
                        right_t_with_dist.append((t, dist))
                    right_t_with_dist.sort(key=lambda x: x[1])  # 按距离排序
                    
                    # 离L_CornerRight近的是T_PenaltyFrontRight
                    if 'T_PenaltyFrontRight' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_PenaltyFrontRight', False):
                        closest_t = right_t_with_dist[0][0]
                        landmarks_2d['T_PenaltyFrontRight'] = (closest_t['pixel_x'], closest_t['pixel_y'])
                        rospy.logdebug("T_PenaltyFrontRight匹配: (%d,%d)", closest_t['pixel_x'], closest_t['pixel_y'])
                    
                    # 另一个是T_GoalFrontRight
                    if len(right_t_with_dist) >= 2 and 'T_GoalFrontRight' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_GoalFrontRight', False):
                        second_t = right_t_with_dist[1][0]
                        landmarks_2d['T_GoalFrontRight'] = (second_t['pixel_x'], second_t['pixel_y'])
                        rospy.logdebug("T_GoalFrontRight匹配: (%d,%d)", second_t['pixel_x'], second_t['pixel_y'])
            
            # 处理左侧T型交点（对应的镜像逻辑）
            if len(left_t) >= 2:
                if 'P_GoalLeft' in landmarks_2d:
                    # 有左门柱时，基于与门柱的距离分配
                    goal_pos = landmarks_2d['P_GoalLeft']
                    left_t_with_dist = []
                    for t in left_t:
                        dist = np.hypot(t['pixel_x'] - goal_pos[0], t['pixel_y'] - goal_pos[1])
                        left_t_with_dist.append((t, dist))
                    left_t_with_dist.sort(key=lambda x: x[1])  # 按距离排序
                    
                    # 离左门柱近的是T_GoalFrontLeft
                    if 'T_GoalFrontLeft' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_GoalFrontLeft', False):
                        closest_t = left_t_with_dist[0][0]
                        landmarks_2d['T_GoalFrontLeft'] = (closest_t['pixel_x'], closest_t['pixel_y'])
                        rospy.logdebug("T_GoalFrontLeft匹配: (%d,%d)", closest_t['pixel_x'], closest_t['pixel_y'])
                    
                    # 另一个是T_PenaltyFrontLeft
                    if len(left_t_with_dist) >= 2 and 'T_PenaltyFrontLeft' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_PenaltyFrontLeft', False):
                        second_t = left_t_with_dist[1][0]
                        landmarks_2d['T_PenaltyFrontLeft'] = (second_t['pixel_x'], second_t['pixel_y'])
                        rospy.logdebug("T_PenaltyFrontLeft匹配: (%d,%d)", second_t['pixel_x'], second_t['pixel_y'])
                        
                elif 'L_CornerLeft' in landmarks_2d:
                    # 没有门柱但有L_CornerLeft时，基于与L_CornerLeft的距离分配
                    corner_pos = landmarks_2d['L_CornerLeft']
                    left_t_with_dist = []
                    for t in left_t:
                        dist = np.hypot(t['pixel_x'] - corner_pos[0], t['pixel_y'] - corner_pos[1])
                        left_t_with_dist.append((t, dist))
                    left_t_with_dist.sort(key=lambda x: x[1])  # 按距离排序
                    
                    # 离L_CornerLeft近的是T_PenaltyFrontLeft
                    if 'T_PenaltyFrontLeft' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_PenaltyFrontLeft', False):
                        closest_t = left_t_with_dist[0][0]
                        landmarks_2d['T_PenaltyFrontLeft'] = (closest_t['pixel_x'], closest_t['pixel_y'])
                        rospy.logdebug("T_PenaltyFrontLeft匹配: (%d,%d)", closest_t['pixel_x'], closest_t['pixel_y'])
                    
                    # 另一个是T_GoalFrontLeft
                    if len(left_t_with_dist) >= 2 and 'T_GoalFrontLeft' not in landmarks_2d and \
                       self.landmark_matching_config_.get('T_GoalFrontLeft', False):
                        second_t = left_t_with_dist[1][0]
                        landmarks_2d['T_GoalFrontLeft'] = (second_t['pixel_x'], second_t['pixel_y'])
                        rospy.logdebug("T_GoalFrontLeft匹配: (%d,%d)", second_t['pixel_x'], second_t['pixel_y'])
        
        rospy.logdebug("地标匹配完成，共匹配到 %d 个地标: %s", 
                       len(landmarks_2d), list(landmarks_2d.keys()))
        
        # ======== 简化的后处理部分：直接在ExtendedObject中设置标签 ========
        # 根据landmarks_2d中的像素坐标，找到对应的原始对象，设置其标签
        for landmark_id, (pixel_x, pixel_y) in landmarks_2d.items():
            # 找到对应的原始对象
            for det_obj in detected_objects:
                if det_obj['pixel_x'] == pixel_x and det_obj['pixel_y'] == pixel_y:
                    original_obj = det_obj['obj_ref']
                    # 设置标签
                    original_obj.label = landmark_id
                    break
        
        rospy.logdebug("使用水平相机匹配到 %d 个地标，已设置到对象的label属性中", len(landmarks_2d))

    def visualize_detections_with_labels(self, rgb_image, merged_objects, landmarks_matched, camera_offset_robot_frame):
        """
        在图像上可视化检测结果（第一个窗口）
        显示：检测框、置信度（简化版，不显示坐标）
        """
        vis_image = rgb_image.copy()
        
        # 定义颜色
        colors = {
            'Post': (255, 165, 0),   # 橙色 - 门柱
            'X': (0, 255, 0),       # 绿色 - X型交点
            'L': (255, 0, 0),       # 蓝色 - L型交点  
            'T': (0, 0, 255),       # 红色 - T型交点
            'Ball': (255, 255, 0)   # 青色 - 球
        }
        
        # 直接使用ExtendedObject中存储的像素信息进行可视化
        for obj in merged_objects:
            # 检查是否来自当前相机
            if (np.array_equal(camera_offset_robot_frame, self.camera_offset_cam1_robot_frame_) and obj.camera_id == 1) or \
               (np.array_equal(camera_offset_robot_frame, self.camera_offset_cam2_robot_frame_) and obj.camera_id == 2):
                
                # 获取颜色
                color = colors.get(obj.type, (255, 255, 255))
                
                # 绘制检测框（使用存储的像素信息）
                cv2.rectangle(vis_image, (obj.bbox_x1, obj.bbox_y1), (obj.bbox_x2, obj.bbox_y2), color, 2)
                
                # 简化的文本信息（只显示类型和置信度）
                text = f"{obj.type} {obj.confidence:.2f}"
                
                # 计算文本大小和位置
                (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                
                # 文本位置
                text_x = obj.bbox_x1
                text_y = obj.bbox_y1 - 10
                
                # 确保文本不超出图像边界
                if text_y - text_height < 0:
                    text_y = obj.bbox_y2 + text_height + 10
                
                # 绘制文本背景
                cv2.rectangle(vis_image, 
                            (text_x, text_y - text_height - 2), 
                            (text_x + text_width + 4, text_y + 2), 
                            (0, 0, 0), -1)
                
                # 绘制文本
                cv2.putText(vis_image, text, (text_x + 2, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        
        # 绘制坐标轴
        origin = (int(self.cx_), int(self.cy_))
        self.draw_axes(vis_image, origin)
        
        return vis_image

    def detect_and_track(self):
        """检测和跟踪目标的主循环"""
        rate = rospy.Rate(10)
        rospy.loginfo("开始目标检测和定位")
        while not rospy.is_shutdown():
            # 清空之前的检测结果
            self.camera1_objects_ = []
            self.camera2_objects_ = []
            
            # 检查相机1的图像(倾斜相机)
            if self.rgb_image_cam1_ is not None and self.depth_image_cam1_ is not None:
                self.camera1_objects_ = self.process_detection(self.rgb_image_cam1_, self.depth_image_cam1_,
                                      self.camera_offset_cam1_robot_frame_, self.camera_orientation_cam1_,
                                      ["Ball", "Post", "X", "L", "T"])
                
                # 为相机1的对象计算真值（如果有标签的话）
                self.calculate_true_values_for_objects(self.camera1_objects_, 
                                                      self.camera_offset_cam1_robot_frame_, 
                                                      self.camera_orientation_cam1_)
                
                # 输出相机1的详细检测日志
                if self.camera1_objects_:
                    self.log_object_details(self.camera1_objects_, "倾斜相机")
            else:
                if self.rgb_image_cam1_ is None:
                    rospy.logdebug("相机1的RGB图像尚未接收")
                if self.depth_image_cam1_ is None:
                    rospy.logdebug("相机1的深度图像尚未接收")
                    
            # 检查相机2的图像（水平相机）
            if self.rgb_image_cam2_ is not None and self.depth_image_cam2_ is not None:
                self.camera2_objects_ = self.process_detection(self.rgb_image_cam2_, self.depth_image_cam2_,
                                      self.camera_offset_cam2_robot_frame_, self.camera_orientation_cam2_,
                                      ["Ball", "Post", "X", "L", "T"])
            else:
                if self.rgb_image_cam2_ is None:
                    rospy.logdebug("相机2的RGB图像尚未接收")
                if self.depth_image_cam2_ is None:
                    rospy.logdebug("相机2的深度图像尚未接收")
            
            # 执行定位（只使用水平相机的检测结果）
            if self.camera2_objects_:
                rospy.loginfo("开始地标匹配流程：使用水平相机检测结果进行地标匹配")
                
                # 使用水平相机的检测结果进行地标匹配（结果存储在对象的label属性中）
                self.match_landmarks(self.camera2_objects_)
                
                # 计算真值坐标（为已匹配标签的对象）
                self.calculate_true_values_for_objects(self.camera2_objects_, 
                                                      self.camera_offset_cam2_robot_frame_, 
                                                      self.camera_orientation_cam2_)
                
                # 输出详细的检测日志
                self.log_object_details(self.camera2_objects_, "水平相机")
                
                # 从已标记的对象中收集地标信息
                labeled_objects = [obj for obj in self.camera2_objects_ if obj.label]
                
                if labeled_objects and len(labeled_objects) >= 1:
                    rospy.loginfo("地标匹配成功，匹配到 %d 个地标，开始在机器人坐标系下进行最小二乘优化", len(labeled_objects))
                    
                    # 首先过滤深度大于阈值的地标
                    remaining_objects = [obj for obj in labeled_objects if obj.x_camera_frame <= self.depth_filter_threshold_]
                    # 过滤掉的数量
                    filtered_out_count = len(labeled_objects) - len(remaining_objects)
                    
                    # 总是输出过滤状态信息
                    rospy.loginfo("深度过滤结果: 原始地标 %d 个，过滤掉 %d 个深度大于%.1fm的地标，剩余 %d 个地标用于优化", 
                                 len(labeled_objects), filtered_out_count, self.depth_filter_threshold_, len(remaining_objects))
                    
                    # 如果有地标被过滤掉，记录详细信息
                    if filtered_out_count > 0:
                        filtered_objects = [obj for obj in labeled_objects if obj.x_camera_frame > self.depth_filter_threshold_]
                        rospy.loginfo("被过滤的地标详情:")
                        for obj in filtered_objects:
                            rospy.loginfo("  过滤地标: %s, 深度: %.3fm", obj.label, obj.x_camera_frame)
                    
                    # 检查过滤后是否还有足够的地标
                    if len(remaining_objects) < 1:
                        rospy.logwarn("深度过滤后可用地标不足（需要至少1个），跳过定位")
                        continue
                    
                    # 使用深度过滤后的地标进行后续选择
                    labeled_objects = remaining_objects
                    
                    # 选择x坐标最小的地标进行优化
                    if len(labeled_objects) > self.max_landmarks_for_optimization_:
                        # 按x坐标排序，选择x坐标最小的指定数量个
                        labeled_objects_sorted = sorted(labeled_objects, key=lambda obj: obj.x_robot_frame)
                        selected_objects = labeled_objects_sorted[:self.max_landmarks_for_optimization_]
                        rospy.loginfo("从 %d 个深度过滤后的地标中选择x坐标最小的 %d 个进行优化", len(labeled_objects), self.max_landmarks_for_optimization_)
                        rospy.loginfo("选中的地标: %s", [obj.label for obj in selected_objects if obj.label])
                        
                        # 详细打印选中地标的信息
                        rospy.loginfo("===== 选中地标详细信息 =====")
                        for i, obj in enumerate(selected_objects):
                            rospy.loginfo("地标 %d:", i + 1)
                            rospy.loginfo("  类型: %s", obj.type)
                            rospy.loginfo("  标签: %s", obj.label if obj.label else "无标签")
                            rospy.loginfo("  机器人坐标系: (%.3f, %.3f, %.3f)", obj.x_robot_frame, obj.y_robot_frame, obj.z_robot_frame)
                            rospy.loginfo("  相机坐标系: (%.3f, %.3f, %.3f)", obj.x_camera_frame, obj.y_camera_frame, obj.z_camera_frame)
                            rospy.loginfo("  像素坐标: (%d, %d)", obj.pixel_x, obj.pixel_y)
                            rospy.loginfo("  置信度: %.3f", obj.confidence)
                            if obj.label and obj.label in self.field_markers_global_frame_:
                                position_global_frame = self.field_markers_global_frame_[obj.label]
                                rospy.loginfo("  场地真实位置: (%.3f, %.3f, %.3f)", position_global_frame[0], position_global_frame[1], position_global_frame[2])
                        rospy.loginfo("=============================")
                    else:
                        selected_objects = labeled_objects
                        rospy.loginfo("使用所有 %d 个地标进行优化", len(labeled_objects))
                        
                        # 详细打印所有地标的信息
                        rospy.loginfo("===== 使用地标详细信息 =====")
                        for i, obj in enumerate(selected_objects):
                            rospy.loginfo("地标 %d:", i + 1)
                            rospy.loginfo("  类型: %s", obj.type)
                            rospy.loginfo("  标签: %s", obj.label if obj.label else "无标签")
                            rospy.loginfo("  机器人坐标系: (%.3f, %.3f, %.3f)", obj.x_robot_frame, obj.y_robot_frame, obj.z_robot_frame)
                            rospy.loginfo("  相机坐标系: (%.3f, %.3f, %.3f)", obj.x_camera_frame, obj.y_camera_frame, obj.z_camera_frame)
                            rospy.loginfo("  像素坐标: (%d, %d)", obj.pixel_x, obj.pixel_y)
                            rospy.loginfo("  置信度: %.3f", obj.confidence)
                            if obj.label and obj.label in self.field_markers_global_frame_:
                                position_global_frame = self.field_markers_global_frame_[obj.label]
                                rospy.loginfo("  场地真实位置: (%.3f, %.3f, %.3f)", position_global_frame[0], position_global_frame[1], position_global_frame[2])
                        rospy.loginfo("=============================")
                    
                    # 准备对应点集用于最小二乘估计（使用机器人坐标系坐标）
                    P_global_frame = []
                    P_robot_frame = []
                    
                    for obj in selected_objects:
                        if obj.label in self.field_markers_global_frame_:
                            P_global_frame.append(self.field_markers_global_frame_[obj.label])
                            P_robot_frame.append([obj.x_robot_frame, obj.y_robot_frame, obj.z_robot_frame])
                    
                    if len(P_global_frame) >= 1:
                        P_global_frame = np.array(P_global_frame)
                        P_robot_frame = np.array(P_robot_frame)
                        
                        # 使用所有匹配的地标进行定位
                        rospy.loginfo("使用所有匹配的%d个地标进行定位", len(P_global_frame))
                        
                        # 记录使用的标记类型
                        marker_types = [obj.label.split('_')[0] for obj in selected_objects if obj.label]
                        rospy.loginfo("使用的标记类型: %s", ', '.join(set(marker_types)))
                        
                        # 估计机器人位姿
                        R, T = self.estimate_pose(P_global_frame, P_robot_frame)
                        
                        if R is not None and T is not None:
                            # 直接使用估计的位姿结果
                            self.robot_position_global_frame_estimated_ = T
                            self.robot_orientation_global_frame_estimated_ = R
                            
                            # 发布机器人2D位姿(x,y,yaw)
                            self.publish_robot_pose_2d()
                            
                            # 发布球和球门的全局位置
                            self.publish_ball_and_goal_global_positions()
                            
                            # 打印定位结果
                            rospy.loginfo("====== 定位结果 ======")
                            rospy.loginfo("匹配的地标: %s", [obj.label for obj in selected_objects])
                            rospy.loginfo("估计位姿: (%.3f, %.3f, %.3f)", T[0], T[1], T[2])
                            
                            if self.robot_position_global_frame_true_ is not None:
                                # 只计算2D位置误差（x、y），忽略z坐标
                                error = np.linalg.norm(T[:2] - 
                                                     np.array([self.robot_position_global_frame_true_.x,
                                                              self.robot_position_global_frame_true_.y]))
                                rospy.loginfo("真实位置: (%.3f, %.3f)", 
                                             self.robot_position_global_frame_true_.x,
                                             self.robot_position_global_frame_true_.y)
                                rospy.loginfo("2D定位误差: %.3f 米", error)
                            rospy.loginfo("===================")
                            rospy.loginfo("完成机器人位姿估计，使用了%d个标记点", len(P_global_frame))
                            
                            # 输出详细的机器人位姿信息到log
                            self.log_robot_pose_info()
                        else:
                            rospy.logerr("位姿估计失败")
                    else:
                        rospy.logwarn("匹配的标记与已知场地标记对应点不足")
                else:
                    rospy.logwarn("水平相机匹配的标记数量不足，至少需要1个标记")
            
            # 打印信息
            print("--------------------------------")
            print(f"相机1检测到的对象数量：{len(self.camera1_objects_)}")
            for obj in self.camera1_objects_:
                print(f"  类型：{obj.type}, 位置: ({obj.x_robot_frame:.3f}, {obj.y_robot_frame:.3f}, {obj.z_robot_frame:.3f})")
            print(f"相机2检测到的对象数量：{len(self.camera2_objects_)}")
            for obj in self.camera2_objects_:
                print(f"  类型：{obj.type}, 位置: ({obj.x_robot_frame:.3f}, {obj.y_robot_frame:.3f}, {obj.z_robot_frame:.3f})")
            print("使用水平相机（相机2）进行地标匹配和定位")
            print("--------------------------------")
            print("\n\n")

            # 显示带有详细标注的图像
            if self.rgb_image_cam1_ is not None:
                try:
                    # 使用新的可视化方法
                    annotated_image_cam1 = self.visualize_detections_with_labels(
                        self.rgb_image_cam1_, self.camera1_objects_, {}, self.camera_offset_cam1_robot_frame_)
                    cv2.imshow(self.window_name_cam1_, annotated_image_cam1)
                except Exception as e:
                    rospy.logerr("相机1图像显示失败: %s", e)
                    
            if self.rgb_image_cam2_ is not None:
                try:
                    # 使用新的可视化方法
                    annotated_image_cam2 = self.visualize_detections_with_labels(
                        self.rgb_image_cam2_, self.camera2_objects_, {}, self.camera_offset_cam2_robot_frame_)
                    cv2.imshow(self.window_name_cam2_detection_, annotated_image_cam2)
                    
                    # 保留原来的地标信息窗口
                    if self.camera2_objects_:
                        vis_image_landmarks = self.visualize_localization(self.rgb_image_cam2_, {})
                        cv2.imshow(self.window_name_cam2_landmarks_, vis_image_landmarks)
                    else:
                        # 如果没有检测到地标，显示原始图像
                        vis_image_landmarks = self.rgb_image_cam2_.copy()
                        origin2 = (int(self.cx_), int(self.cy_))
                        self.draw_axes(vis_image_landmarks, origin2)
                        cv2.imshow(self.window_name_cam2_landmarks_, vis_image_landmarks)
                except Exception as e:
                    rospy.logerr("相机2图像显示失败: %s", e)

            key = cv2.waitKey(1)
            if key == 27:
                break
            rate.sleep()
        cv2.destroyAllWindows()

    def visualize_localization(self, image, landmarks_matched):
        """
        在图像上可视化地标定位结果（第二个窗口）
        显示：中心点、地标标签（简化版，不显示坐标）
        """
        vis_image = image.copy()
        
        # 定义颜色
        colors = {
            'Post': (255, 165, 0),   # 橙色 - 门柱
            'X': (0, 255, 0),        # 绿色 - X型交点
            'L': (255, 0, 0),        # 蓝色 - L型交点  
            'T': (0, 0, 255),        # 红色 - T型交点
            'Ball': (255, 255, 0)    # 青色 - 球
        }
        
        # 为水平相机的对象绘制中心点和标签信息
        for obj in self.camera2_objects_:
            # 获取颜色
            color = colors.get(obj.type, (255, 255, 255))
            
            # 绘制中心点（圆圈）
            cv2.circle(vis_image, (obj.pixel_x, obj.pixel_y), 8, color, 2)
            cv2.circle(vis_image, (obj.pixel_x, obj.pixel_y), 3, color, -1)
            
            # 简化的文本信息（只显示类型和标签）
            if obj.label:
                text = f"{obj.type}: {obj.label}"
            else:
                text = f"{obj.type}"
            
            # 计算文本大小和位置
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            
            # 文本位置（在中心点旁边）
            text_x = obj.pixel_x + 15
            text_y = obj.pixel_y
            
            # 确保文本不超出图像边界
            if text_x + text_width > vis_image.shape[1]:
                text_x = obj.pixel_x - text_width - 15
            if text_y - text_height < 0:
                text_y = obj.pixel_y + text_height + 10
            elif text_y + text_height > vis_image.shape[0]:
                text_y = obj.pixel_y - 10
            
            # 绘制文本背景
            cv2.rectangle(vis_image, 
                        (text_x - 2, text_y - text_height - 2), 
                        (text_x + text_width + 2, text_y + 2), 
                        (0, 0, 0), -1)
            
            # 绘制文本
            cv2.putText(vis_image, text, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        
        # 显示定位结果摘要
        if self.robot_position_global_frame_estimated_ is not None:
            text = f"Est Pos: ({self.robot_position_global_frame_estimated_[0]:.2f}, {self.robot_position_global_frame_estimated_[1]:.2f})"
            cv2.putText(vis_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 显示估计yaw角（如果有的话）
            if self.robot_orientation_global_frame_estimated_ is not None:
                from scipy.spatial.transform import Rotation
                euler_angles = Rotation.from_matrix(self.robot_orientation_global_frame_estimated_).as_euler('xyz', degrees=True)
                text = f"Est Yaw: {euler_angles[2]:.1f} deg"
                cv2.putText(vis_image, text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        if self.robot_position_global_frame_true_ is not None:
            text = f"True Pos: ({self.robot_position_global_frame_true_.x:.2f}, {self.robot_position_global_frame_true_.y:.2f}, {self.robot_position_global_frame_true_.z:.2f})"
            cv2.putText(vis_image, text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 显示真实yaw角
            if self.robot_orientation_global_frame_true_ is not None:
                from scipy.spatial.transform import Rotation
                robot_quat = [self.robot_orientation_global_frame_true_.x, self.robot_orientation_global_frame_true_.y, 
                             self.robot_orientation_global_frame_true_.z, self.robot_orientation_global_frame_true_.w]
                euler_angles = Rotation.from_quat(robot_quat).as_euler('xyz', degrees=True)
                text = f"True Yaw: {euler_angles[2]:.1f} deg"
                cv2.putText(vis_image, text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 计算2D位置误差
            if self.robot_position_global_frame_estimated_ is not None:
                pos_error = np.linalg.norm(self.robot_position_global_frame_estimated_[:2] - 
                                         np.array([self.robot_position_global_frame_true_.x, 
                                                  self.robot_position_global_frame_true_.y]))
                text = f"Pos Error: {pos_error:.3f}m"
                cv2.putText(vis_image, text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # 计算yaw角误差（如果有估计姿态的话）
                if self.robot_orientation_global_frame_estimated_ is not None:
                    # 计算yaw角误差
                    from scipy.spatial.transform import Rotation
                    true_quat = [self.robot_orientation_global_frame_true_.x, self.robot_orientation_global_frame_true_.y, 
                                self.robot_orientation_global_frame_true_.z, self.robot_orientation_global_frame_true_.w]
                    true_rot = Rotation.from_quat(true_quat)
                    est_rot = Rotation.from_matrix(self.robot_orientation_global_frame_estimated_)
                    
                    # 获取yaw角
                    true_yaw = true_rot.as_euler('xyz', degrees=True)[2]
                    est_yaw = est_rot.as_euler('xyz', degrees=True)[2]
                    
                    # 计算yaw角误差（考虑角度周期性）
                    yaw_error = abs(est_yaw - true_yaw)
                    if yaw_error > 180:
                        yaw_error = 360 - yaw_error
                    
                    text = f"Yaw Error: {yaw_error:.1f} deg"
                    cv2.putText(vis_image, text, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 显示已匹配地标的统计信息
        labeled_objects = [obj for obj in self.camera2_objects_ if obj.label]
        if labeled_objects:
            text = f"Matched: {len(labeled_objects)} landmarks"
            cv2.putText(vis_image, text, (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 绘制坐标轴
        origin = (int(self.cx_), int(self.cy_))
        self.draw_axes(vis_image, origin)
        
        return vis_image

    def log_robot_pose_info(self):
        """在log中详细打印机器人2D位姿信息（x、y、yaw）"""
        rospy.loginfo("=============== 机器人2D位姿信息 ===============")
        
        # 估计位姿
        if self.robot_position_global_frame_estimated_ is not None:
            rospy.loginfo("估计位置: (%.3f, %.3f)", 
                         self.robot_position_global_frame_estimated_[0], 
                         self.robot_position_global_frame_estimated_[1])
            
            if self.robot_orientation_global_frame_estimated_ is not None:
                from scipy.spatial.transform import Rotation
                euler_angles = Rotation.from_matrix(self.robot_orientation_global_frame_estimated_).as_euler('xyz', degrees=True)
                rospy.loginfo("估计yaw角: %.1f°", euler_angles[2])  # 只显示yaw角
        else:
            rospy.loginfo("估计位姿: 未获得")
        
        # 真实位姿
        if self.robot_position_global_frame_true_ is not None:
            rospy.loginfo("真实位置: (%.3f, %.3f, %.3f)", 
                         self.robot_position_global_frame_true_.x, 
                         self.robot_position_global_frame_true_.y,
                         self.robot_position_global_frame_true_.z)
            
            if self.robot_orientation_global_frame_true_ is not None:
                from scipy.spatial.transform import Rotation
                robot_quat = [self.robot_orientation_global_frame_true_.x, self.robot_orientation_global_frame_true_.y, 
                             self.robot_orientation_global_frame_true_.z, self.robot_orientation_global_frame_true_.w]
                euler_angles = Rotation.from_quat(robot_quat).as_euler('xyz', degrees=True)
                rospy.loginfo("真实yaw角: %.1f°", euler_angles[2])  # 只显示yaw角
        else:
            rospy.loginfo("真实位姿: 未获得")
        
        # 误差计算
        if (self.robot_position_global_frame_estimated_ is not None and 
            self.robot_position_global_frame_true_ is not None):
            
            # 只计算2D位置误差（x、y）
            pos_error = np.linalg.norm(self.robot_position_global_frame_estimated_[:2] - 
                                     np.array([self.robot_position_global_frame_true_.x, 
                                              self.robot_position_global_frame_true_.y]))
            rospy.loginfo("2D位置误差: %.3f 米", pos_error)
            
            # 只计算yaw角误差
            if (self.robot_orientation_global_frame_estimated_ is not None and 
                self.robot_orientation_global_frame_true_ is not None):
                from scipy.spatial.transform import Rotation
                true_quat = [self.robot_orientation_global_frame_true_.x, self.robot_orientation_global_frame_true_.y, 
                            self.robot_orientation_global_frame_true_.z, self.robot_orientation_global_frame_true_.w]
                true_rot = Rotation.from_quat(true_quat)
                est_rot = Rotation.from_matrix(self.robot_orientation_global_frame_estimated_)
                
                # 获取yaw角
                true_yaw = true_rot.as_euler('xyz', degrees=True)[2]
                est_yaw = est_rot.as_euler('xyz', degrees=True)[2]
                
                # 计算yaw角误差（考虑角度周期性）
                yaw_error = abs(est_yaw - true_yaw)
                if yaw_error > 180:
                    yaw_error = 360 - yaw_error
                
                rospy.loginfo("yaw角误差: %.1f 度", yaw_error)
        
        rospy.loginfo("===============================================")

    def _convert_camera_id_to_type(self, camera_id):
        """
        将数字相机ID转换为CameraType枚举（向后兼容性）
        :param camera_id: 相机ID，1表示俯视相机，2表示水平相机
        :return: CameraType枚举值或None
        """
        if camera_id == 1:
            return CameraType.OVERHEAD_CAMERA
        elif camera_id == 2:
            return CameraType.HORIZONTAL_CAMERA
        else:
            return None

if __name__ == '__main__':
    rospy.init_node('robot_localizer', anonymous=True)
    localizer = RobotLocalizer()
    
    # 使用示例：动态修改检测置信度阈值
    localizer.set_confidence_threshold(0.6) 
    
    # # 使用示例：设置坐标计算模式
    # # 单独设置模式
    # localizer.set_coordinate_calculation_mode(CoordinateMode.DEPTH_MODE, camera_type=CameraType.OVERHEAD_CAMERA)     # 俯视相机使用深度信息模式
    # localizer.set_coordinate_calculation_mode(CoordinateMode.FIXED_Z_MODE, camera_type=CameraType.HORIZONTAL_CAMERA)  
        
    # # 使用示例：设置固定z坐标值（仅在固定z坐标模式下有效）
    # localizer.set_fixed_z_value(-1.11)  # 
    # # localizer.set_fixed_z_value(1.0)   # 设置固定z坐标为1.0米

    # 为两个相机都设置相同模式
    localizer.set_coordinate_calculation_mode(CoordinateMode.DEPTH_MODE)  
     
    localizer.detect_and_track()