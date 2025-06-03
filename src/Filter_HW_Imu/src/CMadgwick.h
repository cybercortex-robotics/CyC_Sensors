// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CMadgwick_H
#define CMadgwick_H

#include "CyC_TYPES.h"
#include "madgwick/Fusion.h"

class CMadgwick
{
public:
    CMadgwick(const bool& _use_magnetometer = false);
    CMadgwick(const CMadgwick&) = default;
    CMadgwick(CMadgwick&&) = default;
    CMadgwick& operator=(const CMadgwick&) = default;
    CMadgwick& operator=(CMadgwick&&) = default;
    ~CMadgwick() = default;

    /**
     * \brief               Updates the Madgwick filter based on the IMU input acc, gyro and magnet (optional) data
     *
     * \param _in_out_imu   Input IMU data acc, gyro and magnet (optional)
     * \param _in_out_imu   Output orientation and quaternions (updated the quat_ variables in _in_out_imu)
     **/
    void update(const float& _dt, CycImu& _in_out_imu);

private:
    FusionAhrs  m_Ahrs;
    bool    m_bUseMagnetometer = false;
};
#endif // CMadgwick_H