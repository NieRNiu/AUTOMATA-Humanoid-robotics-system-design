/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkStates.h>
#include "interface/IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "unitree_legged_msgs/MoveCmd.h"
#include "interface/CmdPanel.h"

class IOROS : public IOInterface
{
public:
    IOROS();
    ~IOROS();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
    void sendCmd(const LowlevelCmd *cmd);
    void recvState(LowlevelState *state);
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub[12], _imu_sub;
    ros::Subscriber _footForce_sub[4];
    ros::Publisher _servo_pub[12];
    ros::Subscriber JoySub;
    ros::Subscriber moveSubscriber;
    unitree_legged_msgs::LowCmd _lowCmd;
    unitree_legged_msgs::LowState _lowState;
    std::string _robot_name;

    float sensitivityIncrease = 0.05;
    float sensitivityDecrease = 0.05;

    // repeated functions for multi-thread
    void initRecv();
    void initSend();

    // Callback functions for ROS
    void imuCallback(const sensor_msgs::Imu &msg);

    void LeftHipPitchCallback(const unitree_legged_msgs::MotorState &msg);
    void LeftHipRollCallback(const unitree_legged_msgs::MotorState &msg);
    void LeftHipYawCallback(const unitree_legged_msgs::MotorState &msg);
    void LeftKneeCallback(const unitree_legged_msgs::MotorState &msg);
    void LeftAnklePitchCallback(const unitree_legged_msgs::MotorState &msg);
    void LeftAnkleRollCallback(const unitree_legged_msgs::MotorState &msg);
    
    void RightHipPitchCallback(const unitree_legged_msgs::MotorState &msg);
    void RightHipRollCallback(const unitree_legged_msgs::MotorState &msg);
    void RightHipYawCallback(const unitree_legged_msgs::MotorState &msg);
    void RightKneeCallback(const unitree_legged_msgs::MotorState &msg);
    void RightAnklePitchCallback(const unitree_legged_msgs::MotorState &msg);
    void RightAnkleRollCallback(const unitree_legged_msgs::MotorState &msg);
    
    void JoystickCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void MoveCallback(const unitree_legged_msgs::MoveCmd &msg);
};

#endif // IOROS_H

#endif // COMPILE_WITH_ROS
