// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImuBrick_API.h"

CImuBrick_API::~CImuBrick_API()
{
    if (isConnected)
    {
        stop();
    }
}

bool CImuBrick_API::setup(int sampleRate)
{
    m_period = 1000 / sampleRate;
    spdlog::info("Connecting to IMU! Period: {}", m_period);
    ipcon_create(&m_connection);

    ipcon_register_callback(&m_connection, IPCON_CALLBACK_ENUMERATE, (void (*)(void))CImuBrick_API::cb_enumerate, (void*)this);
    isConnected = false;

    if (ipcon_connect(&m_connection, DEFAULT_BRICK_HOST, DEFAULT_BRICK_PORT) < 0)
    {
        spdlog::error("Error connecting to IMU");
        return false;
    }

    ipcon_enumerate(&m_connection);
    return true;
}

void CImuBrick_API::cb_connect(const char *uid)
{
    if (isConnected)
        return;

    imu_v2_create(&m_imu, uid, &m_connection);

    bool hasGotSomeAcceleration = false;
    for (int i = 0; i < 200; i++)
    {
        int16_t ax=0, ay=0, az=0;
        imu_v2_get_acceleration(&m_imu, &ax, &ay, &az);
        hasGotSomeAcceleration |= !(ax == 0 && ay == 0 && az == 0);

        if (hasGotSomeAcceleration)
            break;
    }

    if (!hasGotSomeAcceleration)
    {
        spdlog::warn("Imu TIMEOUT, reset!");
        isConnected = false;
        imu_v2_reset(&m_imu);
        return;
    }
    else
    {
        spdlog::info("Imu probe got valid data");
    }

    imu_v2_register_callback(&m_imu, IMU_V2_CALLBACK_ALL_DATA, (void (*)(void))CImuBrick_API::cb_all_data, (void*)this);
    imu_v2_leds_on(&m_imu);

    spdlog::info("IMU Connection: Success!");
    isConnected = true;
}

void CImuBrick_API::start()
{
    imu_v2_set_all_data_period(&m_imu, m_period);
}

void CImuBrick_API::suspend()
{
    imu_v2_set_all_data_period(&m_imu, 0);
}

void CImuBrick_API::stop()
{
    imu_v2_leds_off(&m_imu);
    ipcon_disconnect(&m_connection);
    imu_v2_destroy(&m_imu);
    isConnected = false;
}

void CImuBrick_API::cb_enumerate(
    const char* uid,
    const char* connected_uid,
    char position,
    uint8_t hardware_version[3],
    uint8_t firmware_version[3],
    uint16_t device_identifier,
    uint8_t enumeration_type,
    void* user_data)
{
    if (user_data == nullptr)
        return;

    CImuBrick_API& imu = *reinterpret_cast<CImuBrick_API*>(user_data);
    imu.cb_connect(uid);
}

void CImuBrick_API::cb_all_data(
    int16_t acceleration[3], int16_t magnetic_field[3],
    int16_t angular_velocity[3], int16_t euler_angle[3],
    int16_t quaternion[4], int16_t linear_acceleration[3],
    int16_t gravity_vector[3], int8_t temperature,
    uint8_t calibration_status, void* user_data)
{
    CImuBrick_API& imu = *reinterpret_cast<CImuBrick_API*>(user_data);
    std::lock_guard<std::mutex> guard(imu.m_mutex);

    imu.m_egoData.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    imu.m_egoData.acceleration.x = acceleration[0] / 100.F;
    imu.m_egoData.acceleration.y = acceleration[1] / 100.F;
    imu.m_egoData.acceleration.z = acceleration[2] / 100.F;
    imu.m_egoData.angular_velocity.x = angular_velocity[0] / 16.F;
    imu.m_egoData.angular_velocity.y = angular_velocity[1] / 16.F;
    imu.m_egoData.angular_velocity.z = angular_velocity[2] / 16.F;
    imu.m_egoData.euler_angle.x = euler_angle[0] / 16.F;
    imu.m_egoData.euler_angle.y = euler_angle[1] / 16.F;
    imu.m_egoData.euler_angle.z = euler_angle[2] / 16.F;
    imu.m_egoData.gravity.x = gravity_vector[0] / 100.F;
    imu.m_egoData.gravity.y = gravity_vector[1] / 100.F;
    imu.m_egoData.gravity.z = gravity_vector[2] / 100.F;
    imu.m_egoData.linear_acceleration.x = linear_acceleration[0] / 100.F;
    imu.m_egoData.linear_acceleration.y = linear_acceleration[1] / 100.F;
    imu.m_egoData.linear_acceleration.z = linear_acceleration[2] / 100.F;
    imu.m_egoData.magnetic_field.x = magnetic_field[0] / 16.F;
    imu.m_egoData.magnetic_field.y = magnetic_field[1] / 16.F;
    imu.m_egoData.magnetic_field.z = magnetic_field[2] / 16.F;
    imu.m_egoData.quaternion.x = quaternion[1] / 16383.F;
    imu.m_egoData.quaternion.y = quaternion[2] / 16383.F;
    imu.m_egoData.quaternion.z = quaternion[3] / 16383.F;
    imu.m_egoData.quaternion.w = quaternion[0] / 16383.F;
    ++imu.m_egoData.iter;
}