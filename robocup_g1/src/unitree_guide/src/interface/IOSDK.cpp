/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"
#include "interface/WirelessHandle.h"
#include <stdio.h>
#include "interface/KeyBoard.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_HEIGHTMAP "rt/utlidar/height_map_array"

IOSDK::IOSDK()
{
    InitLowCmd_dds();
    lowcmd_publisher.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &IOSDK::LowCmdwriteHandler, this); // 500hz
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&IOSDK::LowStateMessageHandler, this, std::placeholders::_1), 1);
    cmdPanel = new WirelessHandle();
    // cmdPanel = new KeyBoard();
    pthread_mutex_init(&lowlevelmutex, NULL);
}

void IOSDK::InitLowCmd_dds()
{
    // _lowCmd.head()[0] = HEAD[0];
    // _lowCmd.head()[1] = HEAD[1];
    // _lowCmd.level_flag() = 0xFF;
    // _lowCmd.gpio() = 0;
    mode_pr_ = Mode::PR;
    mode_machine_ = 0;
    _lowCmd.mode_pr() = static_cast<uint8_t>(mode_pr_);
    _lowCmd.mode_machine() = mode_machine_;
   
    for (int i = 0; i < 12; i++)
    {
        _lowCmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        _lowCmd.motor_cmd()[i].q() = (PosStopF);
        _lowCmd.motor_cmd()[i].kp() = (0);
        _lowCmd.motor_cmd()[i].dq() = (VelStopF);
        _lowCmd.motor_cmd()[i].kd() = (0);
        _lowCmd.motor_cmd()[i].tau() = (0);
    }
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    pthread_mutex_lock(&lowlevelmutex);
    _lowCmd.mode_pr() = static_cast<uint8_t>(mode_pr_);
    _lowCmd.mode_machine() = mode_machine_;
    for (int i(0); i < 12; ++i)
    {
        _lowCmd.motor_cmd()[i].mode() = 1;
        _lowCmd.motor_cmd()[i].q() = cmd->motorCmd[i].q;
        _lowCmd.motor_cmd()[i].dq() = cmd->motorCmd[i].dq;
        _lowCmd.motor_cmd()[i].kp() = cmd->motorCmd[i].Kp;
        _lowCmd.motor_cmd()[i].kd() = cmd->motorCmd[i].Kd;
        _lowCmd.motor_cmd()[i].tau() = cmd->motorCmd[i].tau;
    }

    for (int i(0); i < 12; ++i)
    {
        state->motorState[i].q = _lowState.motor_state()[i].q();
        state->motorState[i].dq = _lowState.motor_state()[i].dq();
        state->motorState[i].ddq = _lowState.motor_state()[i].ddq();
        state->motorState[i].tauEst = _lowState.motor_state()[i].tau_est();
        state->motorState[i].mode = _lowState.motor_state()[i].mode();
    }

    for (int i(0); i < 3; ++i)
    {
        state->imu.quaternion[i] = _lowState.imu_state().quaternion()[i];
        state->imu.gyroscope[i] = _lowState.imu_state().gyroscope()[i];
        state->imu.accelerometer[i] = _lowState.imu_state().accelerometer()[i];
    }

    state->imu.quaternion[3] = _lowState.imu_state().quaternion()[3];
    cmdPanel->receiveHandle(&_lowState);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    pthread_mutex_unlock(&lowlevelmutex);
}

void IOSDK::LowCmdwriteHandler()
{
    _lowCmd.crc() = crc32_core((uint32_t *)&_lowCmd, (sizeof(unitree_hg::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(_lowCmd);
}

void IOSDK::LowStateMessageHandler(const void *message)
{
    _lowState = *(unitree_hg::msg::dds_::LowState_*)message;
    if (mode_machine_ != _lowState.mode_machine()) 
    {
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(_lowState.mode_machine()) << std::endl;
      mode_machine_ = unsigned(_lowState.mode_machine());
    }
}

uint32_t IOSDK::crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

#endif // COMPILE_WITH_REAL_ROBOT