/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <optional>
#include <chrono>
#include <iomanip>
#include <thread>

#include "Utilities/getWorkingDir.h"
#include "FSM/FSMState.h"
#include <MNN/Interpreter.hpp>
#include "Utilities/yaml_parser.h"

class State_locomotion : public FSMState
{
public:
    State_locomotion(CtrlComponents *ctrlComp);
    ~State_locomotion() {}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    std::shared_ptr<MNN::Interpreter> net = nullptr;
    MNN::Session* session = nullptr;
    MNN::Tensor *obs_tensor = nullptr;
    MNN::Tensor *act_tensor = nullptr;

    Eigen::VectorXf obs;
    Eigen::VectorXf stiffness;
    Eigen::VectorXf damping;
    Eigen::VectorXf obs_mean;
    Eigen::VectorXf obs_scales;
    Eigen::VectorXf obs_scaled;
    Eigen::VectorXf act_mean;
    Eigen::VectorXf act_scales;
    Eigen::VectorXf act_scaled;
    Eigen::VectorXf act_temp;
    Eigen::VectorXf obs_current;
    Eigen::VectorXf act_prev;

    long long _startTime;
    bool threadRunning = true;
    std::thread _thread;
    void inference();

    Vec12<double> _q;
    Vec12<double> _qd;
    RotMat<double> _B2G_RotMat, _G2B_RotMat;

    void loadPolicy();
    YAML::Node env_cfg;
    int num_observations,num_actions;
    float dt;
    pthread_mutex_t write_cmd_mutex;
    YamlParser yamlParam_;
    float counter = 0;
    float phase = 0;
};
#endif 
