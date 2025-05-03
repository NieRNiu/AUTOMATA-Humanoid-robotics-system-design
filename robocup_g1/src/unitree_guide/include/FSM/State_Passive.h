/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

class State_Passive : public FSMState
{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
};

#endif // PASSIVE_H