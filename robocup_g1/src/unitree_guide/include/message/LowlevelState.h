/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

struct MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4]; // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];

    IMU()
    {
        for (int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMat<double> getRotMat()
    {
        Quat<double> quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3<double> getAcc()
    {
        Vec3<double> acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3<double> getGyro()
    {
        Vec3<double> gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat<double> getQuat()
    {
        Quat<double> q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu;
    UserCommand userCmd;
    UserValue userValue;
    MotorState motorState[12];

    int footForce[4];

    int *getFootSensorData()
    {
        return footForce;
    }

    Vec34<double> getQ()
    {
        Vec34<double> qLegs;
        for (int i(0); i < 4; ++i)
        {
            qLegs.col(i)(0) = motorState[3 * i].q;
            qLegs.col(i)(1) = motorState[3 * i + 1].q;
            qLegs.col(i)(2) = motorState[3 * i + 2].q;
        }
        return qLegs;
    }

    Vec34<double> getQd()
    {
        Vec34<double> qdLegs;
        for (int i(0); i < 4; ++i)
        {
            qdLegs.col(i)(0) = motorState[3 * i].dq;
            qdLegs.col(i)(1) = motorState[3 * i + 1].dq;
            qdLegs.col(i)(2) = motorState[3 * i + 2].dq;
        }
        return qdLegs;
    }

    Vec34<double> getTau()
    {
        Vec34<double> torqueLegs;
        for (int i(0); i < 4; ++i)
        {
            torqueLegs.col(i)(0) = motorState[3 * i].tauEst;
            torqueLegs.col(i)(1) = motorState[3 * i + 1].tauEst;
            torqueLegs.col(i)(2) = motorState[3 * i + 2].tauEst;
        }
        return torqueLegs;
    }

    RotMat<double> getRotMat()
    {
        return imu.getRotMat();
    }

    Vec3<double> getAcc()
    {
        return imu.getAcc();
    }

    Vec3<double> getGyro()
    {
        return imu.getGyro();
    }

    Vec3<double> getAccGlobal()
    {
        return getRotMat() * getAcc();
    }

    Vec3<double> getGyroGlobal()
    {
        return getRotMat() * getGyro();
    }

    Quat<double> getQuaternion()
    {
        return imu.getQuat();
    }

    double getYaw()
    {
        return rotMatToRPY(getRotMat())(2);
    }

    double getPitch()
    {
        return rotMatToRPY(getRotMat())(1);
    }

    double getRoll()
    {
        return rotMatToRPY(getRotMat())(0);
    }

    double getDYaw()
    {
        return getGyroGlobal()(2);
    }
     double getDRoll()
    {
        return getGyroGlobal()(0);
    }
     double getDPitch()
    {
        return getGyroGlobal()(1);
    }

    Vec3<double> getRPY()
    {
        return rotMatToRPY(getRotMat());
    }
};

#endif // LOWLEVELSTATE_HPP