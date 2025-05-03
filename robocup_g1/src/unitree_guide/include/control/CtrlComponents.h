/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include <string>
#include <iostream>


struct CtrlComponents
{
public:
    CtrlComponents(IOInterface *ioInter) : ioInter(ioInter)
    {
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
    }
    ~CtrlComponents()
    {
        delete lowCmd;
        delete lowState;
        delete ioInter;
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv()
    {
        ioInter->sendRecv(lowCmd, lowState);
    }
private:

};

#endif // CTRLCOMPONENTS_H