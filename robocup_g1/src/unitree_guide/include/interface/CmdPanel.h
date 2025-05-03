/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "message/unitree_joystick.h"
#include "common/enumClass.h"
#include <pthread.h>

struct UserValue
{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    UserValue()
    {
        setZero();
    }
    void setZero()
    {
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
    }
};

class CmdPanel
{
public:
    CmdPanel() {}
    virtual ~CmdPanel() {}
    UserCommand getUserCmd() { return userCmd; }
    UserValue getUserValue() { return userValue; }
    void setPassive() { userCmd = UserCommand::L2_B; }
    void setZero() { userValue.setZero(); }
    void setUserValue_lx(float value_lx) { userValue.lx = value_lx; }
    void setUserValue_ly(float value_ly) { userValue.ly = value_ly; }
    void setUserValue_rx(float value_rx) { userValue.rx = value_rx; }
    void setUserValue(float value_lx, float value_ly, float value_rx) { userValue.lx = value_lx; userValue.ly = value_ly; userValue.rx = value_rx; }
    float getUserValue_lx( ) { return userValue.lx;}
    float getUserValue_ly( ) { return userValue.ly;}
    float getUserValue_rx( ) { return userValue.rx;}
#ifdef COMPILE_WITH_REAL_ROBOT
#ifndef ROBOT_TYPE_G1
    virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};
#else
    virtual void receiveHandle(unitree_hg::msg::dds_::LowState_ *lowState){};
#endif // ROBOT_TYPE_G1
#endif // COMPILE_WITH_REAL_ROBOT
protected:
    virtual void *run(void *arg) { return NULL; }
    UserCommand userCmd;
    UserValue userValue;
};

#endif // CMDPANEL_H