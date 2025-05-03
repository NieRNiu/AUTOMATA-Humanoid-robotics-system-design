/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    : _ctrlComp(ctrlComp)
{
    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.locomotion = new State_locomotion(_ctrlComp);

    initialize();
}

FSM::~FSM()
{
    _stateList.deletePtr();
}

void FSM::initialize()
{
    _currentState = _stateList.passive;
    _currentState->enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();

    if (!checkSafty())
    {
      if(_currentState->needcheckSafty)
        _ctrlComp->ioInter->setPassive();
    }

    if (_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if (_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if (_mode == FSMMode::CHANGE)
    {
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState *FSM::getNextState(FSMStateName stateName)
{
    switch (stateName)
    {
        case FSMStateName::INVALID:
            return _stateList.invalid;
            break;
            
        case FSMStateName::PASSIVE:
            return _stateList.passive;
            break;

        case FSMStateName::FIXEDSTAND:
            return _stateList.fixedStand;
            break;

        case FSMStateName::LOCOMOTION:
            return _stateList.locomotion;
            break;

        default:
            return _stateList.invalid;
            break;
    }
}

bool FSM::checkSafty()
{
    // The angle with z axis less than 60 degree
    if ((_ctrlComp->lowState->getRotMat()(2, 2) < 0.5))
    {
        return false;
    }
    else
    {
        return true;
    }
}