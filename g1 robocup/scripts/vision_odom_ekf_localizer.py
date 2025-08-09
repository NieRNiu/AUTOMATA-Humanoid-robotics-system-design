#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose2D
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation
import threading

class EKFState:
    """EKF状态类，对应C++中的ekf::EKFState"""
    def __init__(self):
        self.time = 0.0
        # 状态向量 [x, y, z, qx, qy, qz, qw]
        self.x = np.zeros(7)
        self.x[6] = 1.0  # 初始化为单位四元数
        # 协方差矩阵
        self.P = np.identity(7) 
    
    def pos(self):
        """获取位置向量"""
        return np.array([self.x[0], self.x[1], 0.0])
    
    def q(self):
        """获取四元数"""
        return np.array([self.x[3], self.x[4], self.x[5], self.x[6]])

class EKFControl:
    """EKF控制输入类，对应C++中的ekf::EKFControl"""
    def __init__(self):
        self.time = 0.0
        # 控制向量 [dx, dy, dz, qx, qy, qz, qw]
        self.u = np.zeros(7)
        self.u[6] = 1.0  # 初始化为单位四元数

    def pos(self):
        """获取位置增量"""
        return np.array([self.u[0], self.u[1], self.u[2]])
    
    def q(self):
        """获取四元数增量"""
        return np.array([self.u[3], self.u[4], self.u[5], self.u[6]])

class PoseEKF:
    """姿态EKF类，对应C++中的ekf::PoseEKF"""
    def __init__(self):
        self.time_ = 0.0
        # 状态向量 [x, y, z, qx, qy, qz, qw] 
        self.x_ = np.zeros(7)
        self.x_[6] = 1.0  # 初始化为单位四元数
        # 协方差矩阵
        self.P_ = np.identity(7)
        # 过程噪声
        self.Qn_ = np.identity(7) * 1e-4
        # 测量噪声
        self.Rn_ = np.identity(7) * 1e-2
    
    def set_process_noise(self, Qn):
        """设置过程噪声矩阵"""
        if Qn.shape == (7, 7):
            self.Qn_ = Qn
    
    def set_measurement_noise(self, Rn):
        """设置测量噪声矩阵"""
        if Rn.shape == (7, 7):
            self.Rn_ = Rn
    
    def init(self, state):
        """初始化EKF状态"""
        self.time_ = state.time
        if len(state.x) == 7:
            self.x_ = state.x
        if state.P.shape == (7, 7):
            self.P_ = state.P
    
    def get_state(self):
        """获取当前EKF状态"""
        state = EKFState()
        state.time = self.time_
        state.x = self.x_.copy()
        state.P = self.P_.copy()
        return state
    
    def predict(self, time, u):
        """
        EKF预测步骤
        参数:
            time: 时间戳
            u: 控制输入向量 [dx, dy, dz, qx, qy, qz, qw]
        """
        dt = time - self.time_
        if dt < 0 or len(u) != 7:
            rospy.logwarn(f"EKF控制输入错误: dt = {dt}, u = {u}")
            return
        
        # 提取当前位置和姿态
        pos = self.x_[:3]
        q_pre = np.array([self.x_[6], self.x_[3], self.x_[4], self.x_[5]])  # [w, x, y, z]格式
        q_pre = q_pre / np.linalg.norm(q_pre)  # 归一化
        
        # 提取控制增量
        delta_pos = u[:3]
        delta_q = np.array([u[6], u[3], u[4], u[5]])  # [w, x, y, z]格式
        delta_q = delta_q / np.linalg.norm(delta_q)  # 归一化
        
        # 应用位置增量
        # 创建旋转对象
        r_pre = Rotation.from_quat([q_pre[1], q_pre[2], q_pre[3], q_pre[0]])  # scipy使用[x,y,z,w]格式
        
        # 旋转位置增量
        rotated_delta = r_pre.apply(delta_pos)
        self.x_[:3] += rotated_delta
        
        # 应用姿态增量
        r_delta = Rotation.from_quat([delta_q[1], delta_q[2], delta_q[3], delta_q[0]])
        q_new = (r_pre * r_delta).as_quat()  # [x, y, z, w]格式
        
        # 更新四元数部分
        self.x_[3] = q_new[0]  # x
        self.x_[4] = q_new[1]  # y
        self.x_[5] = q_new[2]  # z
        self.x_[6] = q_new[3]  # w
        
        # 更新协方差矩阵
        F = np.identity(7)  # 简化的状态转移矩阵
        self.P_ = F @ self.P_ @ F.T + self.Qn_
        
        # 更新时间
        self.time_ = time
    
    def update(self, time, z):
        """
        EKF更新步骤
        参数:
            time: 时间戳
            z: 测量向量 [x, y, z, qx, qy, qz, qw]
        """
        if time < self.time_ or len(z) != 7:
            rospy.logwarn(f"EKF更新错误: dt = {time - self.time_}, z = {z}")
            return
        
        # 处理四元数符号问题
        z_mea = z.copy()
        dot_pro = self.x_[3] * z[3] + self.x_[4] * z[4] + self.x_[5] * z[5] + self.x_[6] * z[6]
        if dot_pro < 0:
            z_mea[3:7] = -z_mea[3:7]  # 反转四元数
        
        # 计算测量残差
        y = z_mea - self.x_
        
        # 计算卡尔曼增益
        S = self.P_ + self.Rn_
        K = self.P_ @ np.linalg.inv(S)
        
        # 更新状态
        self.x_ += K @ y
        
        # 归一化四元数部分
        q_norm = np.linalg.norm(self.x_[3:7])
        if q_norm > 0:
            self.x_[3:7] /= q_norm
        
        # 更新协方差矩阵
        I_K = np.identity(7) - K
        self.P_ = I_K @ self.P_ @ I_K.T + K @ self.Rn_ @ K.T

class PoseLocalization:
    """位姿定位类，对应C++中的PoseLocalization"""
    def __init__(self):
        self.mutex_ = threading.Lock()
        self.ekf_ = PoseEKF()
        self.states_ = []  # 存储EKF状态的队列
        self.controls_ = []  # 存储控制输入的队列
        
        self.odom_inited_ = False
        self.pre_odom_q_ = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]格式
        self.pre_odom_pos_ = np.zeros(3)
        
        self.callback_ = None
    
    def set_callback(self, callback):
        """设置回调函数，当状态更新时调用"""
        self.callback_ = callback
    
    def add_odom_pose(self, time, q, pos):
        """
        添加里程计姿态
        参数:
            time: 时间戳
            q: 四元数 [x, y, z, w]格式
            pos: 位置 [x, y, z]
        """
        with self.mutex_:
            # 创建控制输入向量
            u = np.zeros(7)
            u[6] = 1.0  # 默认为单位四元数
            
            if self.odom_inited_:
                # 计算位置和姿态增量
                # 将输入的四元数转换为[w,x,y,z]格式以便计算
                q_wxyz = np.array([q[3], q[0], q[1], q[2]])
                pre_q_wxyz = self.pre_odom_q_
                
                # 创建旋转对象
                r_pre = Rotation.from_quat([pre_q_wxyz[1], pre_q_wxyz[2], pre_q_wxyz[3], pre_q_wxyz[0]])
                r_current = Rotation.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
                
                # 计算相对旋转
                r_delta = r_pre.inv() * r_current
                delta_q = r_delta.as_quat() # [x, y, z, w]格式 
                
                # 计算相对位移（在前一帧坐标系下）
                delta_pos = r_pre.inv().apply(pos - self.pre_odom_pos_)
                
                # 填充控制向量
                u[:3] = delta_pos
                u[3:6] = delta_q[:3]  # x, y, z
                u[6] = delta_q[3]     # w
            
            # 创建并存储控制输入
            control = EKFControl()
            control.time = time
            control.u = u
            self.controls_.append(control)
            
            # 限制队列大小
            if len(self.controls_) > 1022:
                self.controls_.pop(0)
            
            # 更新前一帧的位置和姿态
            self.pre_odom_pos_ = pos.copy()
            self.pre_odom_q_ = np.array([q[3], q[0], q[1], q[2]])  # 存储为[w,x,y,z]格式
            self.odom_inited_ = True
            
            # 如果EKF已初始化，则进行预测
            if self.states_:
                self.ekf_.predict(time, u)
                self.states_.append(self.ekf_.get_state())
                
                # 调用回调函数
                if self.callback_:
                    self.callback_(self.states_[-1])
    
    def add_vision_pose(self, time, q, pos):
        """
        添加视觉定位姿态
        参数:
            time: 时间戳
            q: 四元数 [x, y, z, w]格式
            pos: 位置 [x, y, z]
        """
        with self.mutex_:
            # 创建测量向量
            z = np.zeros(7)
            z[:3] = pos
            z[3:6] = q[:3]  # x, y, z
            z[6] = q[3]     # w
            
            if not self.states_:
                # 首次初始化EKF
                init_state = EKFState()
                init_state.time = time
                init_state.x = z
                self.ekf_.init(init_state)
                self.states_ = [self.ekf_.get_state()]
            else:
                # 找到时间戳之前的最后一个状态
                state = self.states_[0]
                while self.states_ and self.states_[0].time < time:
                    state = self.states_[0]
                    self.states_.pop(0)
                
                # 重新初始化EKF并更新
                self.ekf_.init(state)
                self.ekf_.update(time, z)
                self.states_ = [state, self.ekf_.get_state()]
            
            # 重新应用时间戳之后的所有控制输入
            while self.controls_ and self.controls_[0].time < time:
                self.controls_.pop(0)
            
            for ctrl in self.controls_:
                self.ekf_.predict(ctrl.time, ctrl.u)
                self.states_.append(self.ekf_.get_state())
    
    def get_state(self):
        """获取最新的EKF状态"""
        with self.mutex_:
            if not self.states_:
                return None
            return self.states_[-1]
    
    def get_pose(self, time):
        """
        获取指定时间的位姿
        参数:
            time: 时间戳
        返回:
            (q, pos): 四元数和位置，如果没有可用的状态则返回None
        """
        with self.mutex_:
            if not self.states_ or time < self.states_[0].time:
                return None, None
            
            if time >= self.states_[-1].time:
                # 使用最新的状态
                state = self.states_[-1]
                q = state.q()  # [qx, qy, qz, qw]
                pos = state.pos()  # [x, y, z]
                return q, pos
            
            # 在状态队列中找到时间戳前后的两个状态
            prev = self.states_[0]
            next_state = self.states_[-1]
            
            for i in range(1, len(self.states_)):
                if self.states_[i].time >= time:
                    next_state = self.states_[i]
                    break
                prev = self.states_[i]
            
            if prev.time >= time:
                return None, None
            
            # 线性插值
            alpha = (time - prev.time) / (next_state.time - prev.time)
            
            # 位置插值
            pos = prev.pos() + alpha * (next_state.pos() - prev.pos())
            
            # 四元数球面线性插值
            q_prev = prev.q()  # [qx, qy, qz, qw]
            q_next = next_state.q()  # [qx, qy, qz, qw]
            
            # 转换为scipy的Rotation对象
            r_prev = Rotation.from_quat([q_prev[0], q_prev[1], q_prev[2], q_prev[3]])
            r_next = Rotation.from_quat([q_next[0], q_next[1], q_next[2], q_next[3]])
            
            # 球面线性插值
            r_interp = Rotation.from_quat(
                Rotation.slerp(r_prev.as_quat(), r_next.as_quat(), [alpha])[0]
            )
            
            # 转换回四元数
            q = r_interp.as_quat()  # [qx, qy, qz, qw]
            
            return q, pos

class VisionOdomEKFLocalizer:
    def __init__(self):
        rospy.init_node('vision_odom_ekf_localizer', anonymous=True)
        
        # 创建位姿定位器
        self.pose_ekf_ = PoseLocalization()
        
        # 设置回调函数
        self.pose_ekf_.set_callback(self.ekf_state_callback)
        
        # 里程计缩放因子（可以根据实际情况调整）
        self.scale_ = 1.0  # 默认不缩放
        
        # 机器人模型名称（在Gazebo中）
        self.robot_model_name_ = "g1_12dof"  # 根据实际情况修改
        self.prev_odom_time_ = None
        
        # 设置EKF参数
        self.set_ekf_parameters()
        
        # 最新的视觉定位结果
        self.latest_vision_pose_time_ = None
        self.latest_vision_position_ = None
        self.latest_vision_orientation_ = None
        self.latest_vision_yaw_ = 0.0
        
        # 最新的里程计结果
        self.latest_odom_pose_time_ = None
        self.latest_odom_position_ = None
        self.latest_odom_orientation_ = None
        self.latest_odom_yaw_ = 0.0
        
        # 最新的EKF融合结果
        self.latest_ekf_pose_time_ = None
        self.latest_ekf_position_ = None
        self.latest_ekf_orientation_ = None
        self.latest_ekf_yaw_ = 0.0
        
        # 发布器
        self.ekf_pose_pub_ = rospy.Publisher('/global_position_ekf', PoseStamped, queue_size=10)
        self.ekf_pose_2d_pub_ = rospy.Publisher('/global_position_2d_ekf', Pose2D, queue_size=10)
        
        # 订阅器
        rospy.Subscriber('/global_position_2d_robot', Pose2D, self.vision_pose_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # 定时器，用于定期打印位姿对比信息
        rospy.Timer(rospy.Duration(1.0), self.print_pose_comparison)
        
        rospy.loginfo("视觉-里程计EKF融合定位节点已启动")
        rospy.loginfo("使用/gazebo/model_states作为里程计数据源")
    
    def set_ekf_parameters(self):
        """设置EKF参数"""
        # 设置过程噪声（里程计噪声）
        # 里程计在短时间内相对准确，但长时间会累积误差
        Qn = np.identity(7)
        Qn[:2, :2] *= 0.005  # x,y位置噪声（较小）
        Qn[2, 2] = 0.01      # z噪声（稍大）
        Qn[3:, 3:] *= 0.001  # 四元数噪声（很小）
        self.pose_ekf_.ekf_.set_process_noise(Qn)
        
        # 设置测量噪声（视觉定位噪声）
        # 视觉定位精度通常较高，但可能有跳变
        Rn = np.identity(7) 
        Rn[:2, :2] *= 0.4   # x,y位置噪声
        Rn[2, 2] = 0.1       # z噪声
        Rn[3:, 3:] *= 0.01   # 四元数噪声
        self.pose_ekf_.ekf_.set_measurement_noise(Rn)
        
        rospy.loginfo("已设置EKF参数：")
        rospy.loginfo("  - 过程噪声(里程计): 位置=0.005, z=0.01")
        rospy.loginfo("  - 测量噪声(视觉): 位置=0.05, yaw=0.1") 
    
    def vision_pose_callback(self, pose_msg):
        """
        处理视觉定位结果
        参数:
            pose_msg: Pose2D消息，包含x, y, theta(yaw)
        """
        # 记录视觉定位结果
        self.latest_vision_pose_time_ = rospy.Time.now().to_sec()
        self.latest_vision_position_ = np.array([pose_msg.x, pose_msg.y, 0.0])
        self.latest_vision_yaw_ = pose_msg.theta
        
        # 从yaw角创建四元数
        q = Rotation.from_euler('z', pose_msg.theta).as_quat()  # [x, y, z, w]格式
        self.latest_vision_orientation_ = q
        
        # 添加到EKF
        self.pose_ekf_.add_vision_pose(self.latest_vision_pose_time_, q, self.latest_vision_position_)
        
        rospy.loginfo(f"收到视觉定位结果: 位置=({pose_msg.x:.3f}, {pose_msg.y:.3f}), yaw={math.degrees(pose_msg.theta):.1f}°")
    
    def model_states_callback(self, msg):
        """
        处理Gazebo模型状态消息作为里程计数据
        参数:
            msg: ModelStates消息
        """
        # 查找机器人模型的索引
        try:
            robot_index = msg.name.index(self.robot_model_name_)
        except ValueError:
            # 如果找不到机器人模型，尝试查找第一个包含'g1'的模型
            robot_indices = [i for i, name in enumerate(msg.name) if 'g1' in name]
            if not robot_indices:
                rospy.logwarn_throttle(5.0, f"在model_states中找不到机器人模型 '{self.robot_model_name_}'")
                return
            robot_index = robot_indices[0]
            # 更新模型名称以便下次使用
            self.robot_model_name_ = msg.name[robot_index]
            rospy.loginfo(f"找到机器人模型: {self.robot_model_name_}")
        
        # 获取当前时间
        current_time = rospy.Time.now().to_sec()
        
        # 提取机器人位置和姿态
        pos_odom = np.array([
            msg.pose[robot_index].position.x,
            msg.pose[robot_index].position.y,
            msg.pose[robot_index].position.z
        ]) * self.scale_  # 应用缩放因子
        
        q_odom = np.array([
            msg.pose[robot_index].orientation.x,
            msg.pose[robot_index].orientation.y,
            msg.pose[robot_index].orientation.z,
            msg.pose[robot_index].orientation.w
        ])
        
        # 保存最新的里程计结果
        self.latest_odom_pose_time_ = current_time
        self.latest_odom_position_ = pos_odom
        self.latest_odom_orientation_ = q_odom
        
        # 计算yaw角
        r_odom = Rotation.from_quat(q_odom)
        euler_angles = r_odom.as_euler('xyz')
        self.latest_odom_yaw_ = euler_angles[2]  # yaw角
        
        # 添加到EKF
        self.pose_ekf_.add_odom_pose(current_time, q_odom, pos_odom)
        
        # 更新上一次时间戳
        self.prev_odom_time_ = current_time
        
        rospy.logdebug(f"处理Gazebo模型状态: 时间={current_time:.3f}, 位置=({pos_odom[0]:.3f}, {pos_odom[1]:.3f}, {pos_odom[2]:.3f})")
    
    def ekf_state_callback(self, state):
        """
        EKF状态更新回调
        参数:
            state: EKFState对象
        """
        # 记录最新的EKF融合结果
        self.latest_ekf_pose_time_ = state.time
        self.latest_ekf_position_ = state.pos()
        self.latest_ekf_orientation_ = state.q()  # [qx, qy, qz, qw]
        
        # 计算yaw角
        r = Rotation.from_quat(self.latest_ekf_orientation_)
        euler_angles = r.as_euler('xyz')
        self.latest_ekf_yaw_ = euler_angles[2]  # yaw角
        
        # 发布融合后的位姿
        self.publish_ekf_pose()
    
    def print_pose_comparison(self, event=None):
        """定期打印位姿对比信息"""
        if (self.latest_ekf_position_ is None or 
            self.latest_odom_position_ is None or 
            self.latest_vision_position_ is None):
            return
        
        # 打印位姿对比信息
        rospy.loginfo("===== 位姿对比信息 =====")
        
        # EKF融合结果
        rospy.loginfo(f"EKF融合: 位置=({self.latest_ekf_position_[0]:.3f}, {self.latest_ekf_position_[1]:.3f}), " +
                     f"yaw={math.degrees(self.latest_ekf_yaw_):.1f}°")
        
        # 里程计结果
        rospy.loginfo(f"里程计: 位置=({self.latest_odom_position_[0]:.3f}, {self.latest_odom_position_[1]:.3f}), " +
                     f"yaw={math.degrees(self.latest_odom_yaw_):.1f}°")
        
        # 视觉定位结果
        rospy.loginfo(f"视觉定位: 位置=({self.latest_vision_position_[0]:.3f}, {self.latest_vision_position_[1]:.3f}), " +
                     f"yaw={math.degrees(self.latest_vision_yaw_):.1f}°")
        
        # 计算误差
        odom_pos_error = np.linalg.norm(self.latest_odom_position_[:2] - self.latest_ekf_position_[:2])
        vision_pos_error = np.linalg.norm(self.latest_vision_position_[:2] - self.latest_ekf_position_[:2])
        
        odom_yaw_error = abs(math.degrees(self.latest_odom_yaw_ - self.latest_ekf_yaw_))
        vision_yaw_error = abs(math.degrees(self.latest_vision_yaw_ - self.latest_ekf_yaw_))
        
        # 处理角度差值，确保在[-180, 180]范围内
        if odom_yaw_error > 180:
            odom_yaw_error = 360 - odom_yaw_error
        if vision_yaw_error > 180:
            vision_yaw_error = 360 - vision_yaw_error
        
        rospy.loginfo(f"里程计误差: 位置={odom_pos_error:.3f}m, yaw={odom_yaw_error:.1f}°")
        rospy.loginfo(f"视觉误差: 位置={vision_pos_error:.3f}m, yaw={vision_yaw_error:.1f}°")
        rospy.loginfo("=======================")
    
    def publish_ekf_pose(self):
        """发布EKF融合后的位姿"""
        if self.latest_ekf_position_ is None or self.latest_ekf_orientation_ is None:
            return
        
        # 创建PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.from_sec(self.latest_ekf_pose_time_)
        pose_msg.header.frame_id = "map"
        
        # 设置位置
        pose_msg.pose.position.x = self.latest_ekf_position_[0]
        pose_msg.pose.position.y = self.latest_ekf_position_[1]
        pose_msg.pose.position.z = self.latest_ekf_position_[2]
        
        # 设置姿态
        pose_msg.pose.orientation.x = self.latest_ekf_orientation_[0]
        pose_msg.pose.orientation.y = self.latest_ekf_orientation_[1]
        pose_msg.pose.orientation.z = self.latest_ekf_orientation_[2]
        pose_msg.pose.orientation.w = self.latest_ekf_orientation_[3]
        
        # 发布PoseStamped消息
        self.ekf_pose_pub_.publish(pose_msg)
        
        # 创建并发布Pose2D消息（x, y, yaw）
        pose_2d_msg = Pose2D()
        pose_2d_msg.x = self.latest_ekf_position_[0]
        pose_2d_msg.y = self.latest_ekf_position_[1]
        pose_2d_msg.theta = self.latest_ekf_yaw_
        
        # 发布Pose2D消息
        self.ekf_pose_2d_pub_.publish(pose_2d_msg)

    def run(self):
        """运行节点的主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 主循环中不需要额外的逻辑，因为我们已经有定时器和回调函数处理所有功能
            rate.sleep()

if __name__ == '__main__':
    try:
        localizer = VisionOdomEKFLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        pass 