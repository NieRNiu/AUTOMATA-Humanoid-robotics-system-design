/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand")
{
    this->needcheckSafty = false;
}

void State_FixedStand::enter()
{
    for(int i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].Kp = kp[i];
        _lowCmd->motorCmd[i].Kd = kd[i];
        _lowCmd->motorCmd[i].tau = 0;
    }
    
    for(int i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
}

void State_FixedStand::run()
{
    _percent += (float)1 / _duration;
    _percent = _percent > 1 ? 1 : _percent;
    if (_percent <= 1)
    {
        for (int j = 0; j < 12; j++)
        {
            _lowCmd->motorCmd[j].q = (1 - _percent) * _startPos[j] + _percent * _targetPos[j];
        }
    }
}

void State_FixedStand::exit()
{
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::START)
    {
        return FSMStateName::LOCOMOTION;
    }
    else
    {
        return FSMStateName::FIXEDSTAND;
    }
}