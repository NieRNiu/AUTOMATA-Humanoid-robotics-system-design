#!/usr/bin/env python

import rospy
from unitree_legged_msgs.msg import MoveCmd
import time

def Move(vx, vy, vyaw, move_time=0.0):
    pub = rospy.Publisher('/g1_12dof_gazebo/move_controller/command', MoveCmd, queue_size=10)
    command = MoveCmd()
    command.vx = vx
    command.vy = vy
    command.vyaw = vyaw

    if move_time > 0:
        start_time = time.time()
        while time.time() - start_time < move_time and not rospy.is_shutdown():
            pub.publish(command)
            rospy.sleep(0.1)

        command.vx = 0
        command.vy = 0
        command.vyaw = 0
        pub.publish(command)
    else:
        pub.publish(command)

    return 0

def patrol_rectangle(edge_time=5, loops=1):
    for i in range(loops):
        rospy.loginfo(f"== moving square {i+1} ==")
        Move(1, 0, 0, edge_time)   # left
        Move(0, -1, 0, edge_time)  # backward
        Move(-1, 0, 0, edge_time)  # right
        Move(0, 1, 0, edge_time)   # forward
        rospy.loginfo(f"== square {i+1} ended ==\n")

def patrol_circle(duration_per_circle=63, loops=1):
    for i in range(loops):
        rospy.loginfo(f"== moving circle {i+1} ==")
        # enable vy & vyaw
        Move(0, 1, 1, duration_per_circle)
        rospy.loginfo(f"== circle {i+1} ended ==\n")

def main():
    rospy.init_node('patrol_controller', anonymous=True)

    try:
        while not rospy.is_shutdown():
            user_input = input("input cruising commend (route and times exp: 'square 1' or 'circle 2'， input 'exit' to quit): ").strip().lower()
            if user_input in ['exit']:
                rospy.loginfo("exit cruise mode")
                break

            if not user_input:
                rospy.logwarn("no commend")
                continue
            
            parts = user_input.split()
            if len(parts) != 2 or not parts[1].isdigit():
                rospy.logwarn("error, should be 'square 1' or 'circle 2'")
                continue

            command, times = parts[0], int(parts[1])

            if command == 'square':
                rospy.loginfo("start cruising...")
                patrol_rectangle(edge_time=5, loops=times)
            elif command == 'circle':
                rospy.loginfo("start cruising...")
                patrol_circle(duration_per_circle=10, loops=times)
            else:
                rospy.logwarn(f"unknown: {command}")
                continue

            rospy.loginfo("Done. Waiting for next commend...")
            Move(0, 0, 0)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
