/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform
{
    GAZEBO,
};

enum class UserCommand
{
    NONE,
    START,
    L2_A,  
    L2_B,  
    L2_X, 
    L2_Y, 
    L1_X,
    L1_A,
    L1_Y,
    L1_B,
    R1_A,
    R1_B,
    R1_X,
    R1_Y,
    R2_A,
    R2_B,
    R2_X,
    R2_Y,
};

enum class FSMMode
{
    NORMAL,
    CHANGE
};

enum class FSMStateName
{
    EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    LOCOMOTION,
};

#endif // ENUMCLASS_H