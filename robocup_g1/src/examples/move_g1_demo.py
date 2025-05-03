#!/usr/bin/env python

import rospy
from unitree_legged_msgs.msg import MoveCmd
import time

def Move(vx, vy, vyaw, move_time=0.0):
    """
    控制机器人移动的函数
    Args:
        vx: 左右方向速度 (正值为前进，负值为后退)
        vy: 前后方向速度 (正值为右移，负值为左移)
        vyaw: 旋转速度 (正值为顺时针旋转，负值为逆时针旋转)
        move_time: 移动持续时间(秒)，0表示持续移动直到下一个命令
    """
    pub = rospy.Publisher('/g1_12dof_gazebo/move_controller/command', MoveCmd, queue_size=10)
    command = MoveCmd()
    command.vx = vx
    command.vy = vy
    command.vyaw = vyaw
       
    if move_time > 0:
        # 如果指定了移动时间，则发布命令并等待指定时间
        start_time = time.time()
        while time.time() - start_time < move_time and not rospy.is_shutdown():
            pub.publish(command)
            rospy.sleep(0.1)  # 100ms interval
        # 发送停止命令
        command.vx = 0
        command.vy = 0
        command.vyaw = 0
        pub.publish(command)
    else:
        # 如果没有指定时间，则只发送一次命令
        pub.publish(command)
    
    return 0  # 返回成功

def main():
    rospy.init_node('move_control', anonymous=True)
    
    # 示例：执行一系列移动命令
    try:     
        rospy.loginfo("*************begin**************")
        
        # left movement
        Move(1, 0, 0, 5)
        rospy.loginfo("************right movement***********")        
        Move(0, 0, 0, 5)  

        # right movement
        Move(-1, 0, 0, 5)
        rospy.loginfo("************left movement***********")
        Move(0, 0, 0, 5) 
        
        # forward movement
        Move(0, 1, 0, 5)
        rospy.loginfo("************forward movement***********")
        Move(0, 0, 0, 5) 

        # right movement
        Move(0, -1, 0, 5)
        rospy.loginfo("************backward movement***********")
        Move(0, 0, 0, 5) 

        # cw movement
        Move(0, 0, 1, 5)
        rospy.loginfo("************cw movement***********")
        Move(0, 0, 0, 5) 
        # ccw movement
        Move(0, 0, -1, 5)
        rospy.loginfo("************ccw movement***********")
        Move(0, 0, 0, 5)           
       
        rospy.loginfo("**********stop***************")
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
