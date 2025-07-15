#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from unitree_legged_msgs.msg import MoveCmd
import time

def Move(vx, vy, vyaw, move_time=0.0):
    """
    Sends motion commands to the robot.

    Args:
        vx (int): Forward/backward velocity. Positive = forward.
        vy (int): Lateral velocity. Not used here.
        vyaw (int): Rotational velocity. Positive = clockwise.
        move_time (float): Duration in seconds. 0 = one-time command.
    """
    pub = rospy.Publisher('/g1_12dof_gazebo/move_controller/command', MoveCmd, queue_size=10)
    rospy.sleep(0.5)  # Allow publisher to initialize

    command = MoveCmd()
    command.vx = int(vx)
    command.vy = int(vy)
    command.vyaw = int(vyaw)

    if move_time > 0:
        start_time = time.time()
        while time.time() - start_time < move_time and not rospy.is_shutdown():
            pub.publish(command)
            rospy.sleep(0.1)
        # Stop the robot after motion
        command.vx = 0
        command.vy = 0
        command.vyaw = 0
        pub.publish(command)
    else:
        pub.publish(command)
    return 0

def circle_motion(duration=15, speed=1, yaw_speed=1):
    """
    Makes the robot walk in a circular path.

    Args:
        duration (int): Total duration in seconds.
        speed (int): Forward speed (vx).
        yaw_speed (int): Rotational speed (vyaw).
    """
    rospy.loginfo("Starting circular motion...")
    Move(speed, 0, yaw_speed, duration)
    rospy.loginfo("Circular motion complete. Stopping robot.")
    Move(0, 0, 0)

def main():
    rospy.init_node('circle_walk', anonymous=True)

    try:
        rospy.loginfo("Starting G1 robot circular patrol...")
        circle_motion(duration=15, speed=1, yaw_speed=1)
        rospy.loginfo("Motion complete.")
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
