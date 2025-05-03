/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include "Utilities/time/timeMarker.h"
#include "unitree/robot/go2/robot_state/robot_state_client.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/common/time/time_tool.hpp"
#include "unitree/common/thread/thread.hpp"
using namespace unitree::common;
using namespace unitree::robot;
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

#ifdef COMPILE_WITH_MOVE_BASE
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#endif // COMPILE_WITH_MOVE_BASE

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

class IOSDK : public IOInterface
{
public:
    IOSDK();
    ~IOSDK() {}
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
    Mode mode_pr_;
    uint8_t mode_machine_;
    uint8_t HEAD[2] = {0xFE, 0xEF};
    pthread_mutex_t lowlevelmutex;
    unitree_hg::msg::dds_::LowCmd_ _lowCmd{};
    unitree_hg::msg::dds_::LowState_ _lowState{};
    unitree::common::ThreadPtr lowCmdWriteThreadPtr;
    unitree::common::ThreadPtr highStateWriteThreadPtr;
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber;
    void LowCmdwriteHandler(); 
    void LowStateMessageHandler(const void *);
    void InitLowCmd_dds();
    uint32_t crc32_core(uint32_t* ptr, uint32_t len);
};

#endif // IOSDK_H