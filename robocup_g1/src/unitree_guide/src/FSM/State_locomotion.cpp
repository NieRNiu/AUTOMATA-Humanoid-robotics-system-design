/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_locomotion.h"
#include <cmath> 

State_locomotion::State_locomotion(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::LOCOMOTION, "LOCOMOTION")
{
    this->needcheckSafty = false;
    int ret1 = pthread_mutex_init(&write_cmd_mutex,NULL);        
    loadPolicy();
}

void State_locomotion::loadPolicy()
{
    std::string policy_file_path;
#ifdef COMPILE_WITH_CATKIN_MAKE 
    #ifdef ROBOT_TYPE_G1
        std::string env_cfg_path = getWorkingDir() + "/src/unitree_guide/data/policy/motion/g1.yaml";
        env_cfg = YAML::LoadFile(env_cfg_path.c_str());
        yamlParam_.setup(env_cfg_path.c_str());
		policy_file_path = getWorkingDir() + "/src/unitree_guide/data/policy/motion/actor_1.mnn";
	#endif
#else 
#endif
    
    std::cout<<"[State_locomotion] policy_file_path: "<<policy_file_path<<std::endl;

    net = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(policy_file_path.c_str()));
    net->setSessionMode(MNN::Interpreter::Session_Debug);

    MNN::ScheduleConfig config;
    config.numThread = 2;
    session = net->createSession(config);

    num_observations = 47;
    num_actions = 12;

    auto tensorMap = net->getSessionInputAll(session);

    std::cout << "Tensors Info:" << std::endl;
    std::cout << "---------------------------------" << std::endl;
    
    for(const auto& [name, tensor] : tensorMap) {
        std::cout << "Tensor Name: " << name << std::endl;
        std::cout << "  Shape: [";
        for(int i=0; i<tensor->dimensions(); i++) {
            std::cout << tensor->length(i);
            if(i < tensor->dimensions()-1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        std::cout << "  Element Size: " << tensor->elementSize() << std::endl;
        std::cout << "---------------------------------" << std::endl;
    }


    obs_tensor = net->getSessionInput(session, "input");
    act_tensor = net->getSessionOutput(session, "output");

    dt = yamlParam_.ReadFloatFromYaml("control_dt");
    stiffness = yamlParam_.ReadVectorFromYaml("kps",num_actions);
    damping = yamlParam_.ReadVectorFromYaml("kds",num_actions);

    obs_scaled.setZero(num_observations);
    obs_current.setZero(num_observations);
    act_scaled.setZero(num_actions);
    act_temp.setZero(num_actions);
    
    obs_mean.setZero(num_observations);
    obs_mean.segment<12>(9) = yamlParam_.ReadVectorFromYaml("default_angles",num_actions);

    float obsScaleAngularVelocity  = yamlParam_.ReadFloatFromYaml("ang_vel_scale");
    float obsScaleDofPosition      = yamlParam_.ReadFloatFromYaml("dof_pos_scale");
    float obsScaleDofVelocity      = yamlParam_.ReadFloatFromYaml("dof_vel_scale");
    float obsScaleLastActions      = yamlParam_.ReadFloatFromYaml("action_scale");

    obs_scales.setZero(num_observations);
    obs_scales.head<3>()       = Eigen::Vector3f::Ones() * obsScaleAngularVelocity;      // Angular velocity 3
    obs_scales.segment<3> ( 3) = Eigen::Vector3f::Ones();     // Projected gravity 3
    obs_scales.segment<3> ( 6) = Eigen::Vector3f::Ones();     // Cmd 3
    // obs_scales.segment<12>( 9) = Eigen::Vector<float, 12>::Ones() * obsScaleDofPosition; // Joint position 12
    // obs_scales.segment<12>(21) = Eigen::Vector<float, 12>::Ones() * obsScaleDofVelocity; // Joint velocity 12
    // obs_scales.segment<12>(33) = Eigen::Vector<float, 12>::Ones(); // Last actions 12
    // obs_scales.segment<2>(45) = Eigen::Vector<float, 2>::Ones(); // Phase 2

    obs_scales.segment<12>( 9) = Eigen::Matrix<float, 12, 1>::Ones() * obsScaleDofPosition; // Joint position 12
    obs_scales.segment<12>(21) = Eigen::Matrix<float, 12, 1>::Ones() * obsScaleDofVelocity; // Joint velocity 12
    obs_scales.segment<12>(33) = Eigen::Matrix<float, 12, 1>::Ones(); // Last actions 12
    obs_scales.segment<2>(45) = Eigen::Matrix<float, 2, 1>::Ones(); // Phase 2

    act_mean.setZero(num_actions);
    act_mean.segment<12>(0) = yamlParam_.ReadVectorFromYaml("default_angles",num_actions);
    
    act_scales.setZero(num_actions);
    // act_scales.segment<12>(0) = Eigen::Vector<float, 12>::Ones() * obsScaleLastActions;

    act_scales.segment<12>(0) = Eigen::Matrix<float, 12, 1>::Ones() * obsScaleLastActions;

    act_prev.setZero(num_actions);
    obs_current.setZero(num_actions);

   std::cout<<"---------------- debug ----------------  "<<std::endl;
   std::cout<<"[State_locomotion::loadPolicy] dt: \n"<< dt <<std::endl;
   std::cout<<"[State_locomotion::loadPolicy] stiffness: \n"<< stiffness <<std::endl;
   std::cout<<"[State_locomotion::loadPolicy] damping: \n"<< damping <<std::endl;
   std::cout<<"[State_locomotion::loadPolicy] obsScaleAngularVelocity: \n"<< obsScaleAngularVelocity <<std::endl;
   std::cout<<"[State_locomotion::loadPolicy] obsScaleDofPosition: \n"<< obsScaleDofPosition <<std::endl;
   std::cout<<"[State_locomotion::loadPolicy] obsScaleDofVelocity: \n"<< obsScaleDofVelocity <<std::endl;
   std::cout<<"[State_locomotion::loadPolicy] obsScaleLastActions: \n"<< obsScaleLastActions <<std::endl;

}

void State_locomotion::enter()
{
    std::cout<<"[State_locomotion] enter "<<std::endl;
    counter = 0;
    threadRunning = true;
    _thread = std::thread(&State_locomotion::inference, this);
}

void State_locomotion::inference()
{
    while(threadRunning)
    {
        _startTime = getSystemTime();

        counter += 1;
        obs_current.setZero(num_observations);

        // Angular velocity 3
        obs_current(0) = _lowState->getGyro()[0];
        obs_current(1) = _lowState->getGyro()[1];
        obs_current(2) = _lowState->getGyro()[2];

        // Projected gravity
        _B2G_RotMat = _lowState->getRotMat();
        _G2B_RotMat = _B2G_RotMat.transpose();
        Vec3<double> projected_gravity_body,projected_gravity_world;
        projected_gravity_world<<0,0,-1;
        projected_gravity_body = _G2B_RotMat * projected_gravity_world;
        obs_current(3) = projected_gravity_body(0);
        obs_current(4) = projected_gravity_body(1);
        obs_current(5) = projected_gravity_body(2);

        _userValue = _lowState->userValue;
        // cmd
        obs_current(6) = _userValue.ly;
        obs_current(7) = -_userValue.lx;
        obs_current(8) = -_userValue.rx;

        for (int j = 0; j < 12; ++j)
        {
            obs_current(9 + j) = _lowState->motorState[j].q;
            obs_current(21 + j) = _lowState->motorState[j].dq;
        }

        obs_current.segment<12>(33) << act_prev;

        float period = 0.8;
        float count = counter * dt;
        float phase = std::fmod(static_cast<float>(count), period) / period;
        float sin_phase = std::sin(2 * M_PI * phase);
        float cos_phase = std::cos(2 * M_PI * phase);

        obs_current(45) = sin_phase;
        obs_current(46) = cos_phase;

        obs_scaled = (obs_current - obs_mean).cwiseProduct(obs_scales);
        
        for (int i = 0; i < num_observations; ++i)
        {
            obs_tensor->host<float>()[i] = obs_scaled(i);
        }
            
        net->runSession(session);
        
        for (int i = 0; i < 12; ++i)
        {
            act_prev(i) = act_tensor->host<float>()[i];
        }

        pthread_mutex_lock(&write_cmd_mutex);
        act_scaled = act_prev.cwiseProduct(act_scales) + act_mean;
        pthread_mutex_unlock(&write_cmd_mutex);
        absoluteWait(_startTime, (long long)(dt * 1000000));
    }
    std::cout<<"end"<<std::endl;
    threadRunning = false;
    std::cout<<"done!"<<std::endl;
}


void State_locomotion::run()
{
    pthread_mutex_lock(&write_cmd_mutex);
    memcpy(act_temp.data(),act_scaled.data(),act_scaled.size()* sizeof(float));
    pthread_mutex_unlock(&write_cmd_mutex);

    for (int i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = act_temp[i];
        _lowCmd->motorCmd[i].Kp = stiffness(i);
        _lowCmd->motorCmd[i].Kd = damping(i);
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].tau = 0;
    }
}

void State_locomotion::exit()
{
    counter = 0;
    act_prev.setZero();
    threadRunning = false;
    _thread.join();
}

FSMStateName State_locomotion::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::LOCOMOTION;
    }
}