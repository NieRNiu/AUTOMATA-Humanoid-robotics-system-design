/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Passive.h"
#include <cstdlib>
#include <csignal>
#include <signal.h>

State_Passive::State_Passive(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::PASSIVE, "passive")
{
    this->needcheckSafty = false;
}

void State_Passive::enter()
{       
        std::cout<<"[FSM] enter State_Passive"<<std::endl;
        for (int i = 0; i < 12; i++)
        {
                _lowCmd->motorCmd[i].q = 0;
                _lowCmd->motorCmd[i].dq = 0;
                _lowCmd->motorCmd[i].Kp = 0;
                _lowCmd->motorCmd[i].Kd = 8;
                _lowCmd->motorCmd[i].tau = 0;
        }
}

void State_Passive::run()
{

}

void State_Passive::exit()
{

}

FSMStateName State_Passive::checkChange()
{
        if (_lowState->userCmd == UserCommand::L2_A)
        {
                return FSMStateName::FIXEDSTAND;
        }
        else
        {
                return FSMStateName::PASSIVE;
        }
}