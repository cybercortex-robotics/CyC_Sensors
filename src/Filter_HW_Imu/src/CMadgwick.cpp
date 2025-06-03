// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CMadgwick.h"

CMadgwick::CMadgwick(const bool& _use_magnetometer) :
    m_bUseMagnetometer(_use_magnetometer)
{
    FusionAhrsInitialise(&m_Ahrs);
}

void CMadgwick::update(const float& _dt, CycImu& _in_out_imu)
{
    const FusionVector gyroscope = { _in_out_imu.gyro.x() * RAD2DEG, _in_out_imu.gyro.y() * RAD2DEG, _in_out_imu.gyro.z() * RAD2DEG };
    const FusionVector accelerometer = { _in_out_imu.acc.x(), _in_out_imu.acc.y(), _in_out_imu.acc.z()};
    const FusionVector magnetometer = { _in_out_imu.magnet.x(), _in_out_imu.magnet.y(), _in_out_imu.magnet.z()};

    if (m_bUseMagnetometer)
        FusionAhrsUpdate(&m_Ahrs, gyroscope, accelerometer, magnetometer, _dt);
    else
        FusionAhrsUpdateNoMagnetometer(&m_Ahrs, gyroscope, accelerometer, _dt);

    FusionQuaternion fusion_quat = FusionAhrsGetQuaternion(&m_Ahrs);

    // Update filter quaternion output to Madgwick estimation
    _in_out_imu.quat.update(fusion_quat.element.x, fusion_quat.element.y, fusion_quat.element.z, fusion_quat.element.w);
}