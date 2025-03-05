// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CImuUtils_H
#define CImuUtils_H

#include "CyC_TYPES.h"

class CImuUtils
{
public:
    enum ImuApiType
    {
        Imu_API_SIM       = 0,
        Imu_API_MPU9250   = 1,
        Imu_API_BRICK     = 2,
        Imu_API_ANDROID   = 3,
        Imu_API_UNITREEA1 = 4,
        Imu_API_UNITY     = 5,
        Imu_API_REALSENSE = 6
    };
    
public:
    CImuUtils() = default;
    CImuUtils(const CImuUtils&) = default;
    CImuUtils(CImuUtils&&) = default;
    CImuUtils& operator=(const CImuUtils&) = default;
    CImuUtils& operator=(CImuUtils&&) = default;
    ~CImuUtils() = default;
};
#endif // CImuUtils_H