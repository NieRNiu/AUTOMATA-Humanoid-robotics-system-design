/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

#include "common/enumClass.h"
#include "control/CtrlComponents.h"
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_locomotion.h"

struct FSMStateList
{
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    State_locomotion *locomotion;

    void deletePtr()
    {
        delete invalid;
        delete passive;
        delete fixedStand;
        delete locomotion;
    }
};

class FSM
{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run(); 

private:
    FSMState *getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};

#endif // FSM_H