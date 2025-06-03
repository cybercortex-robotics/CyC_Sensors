// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CImuBrick_API_H
#define CImuBrick_API_H

#define IPCON_EXPOSE_INTERNALS

#include "CyC_TYPES.h"
#include <cmath>
#include "ip_connection.h"
#include "brick_imu_v2.h"

#define DEFAULT_BRICK_HOST "localhost"
#define DEFAULT_BRICK_PORT 4223

class CImuBrick_API
{
public:
    struct triaxis
    {
        float x;
        float y;
        float z;
    };

    struct quadaxis
    {
        float x;
        float y;
        float z;
        float w;
    };

    struct ego_data
    {
        triaxis acceleration;
        triaxis magnetic_field;
        triaxis angular_velocity;
        triaxis euler_angle;
        triaxis linear_acceleration;
        triaxis gravity;
        quadaxis quaternion;
        size_t iter = 0;
        size_t timestamp = 0; // us
    };

    CImuBrick_API() = default;
    ~CImuBrick_API();
    CImuBrick_API(const CImuBrick_API&) = delete;
    CImuBrick_API(CImuBrick_API&&) = delete;
    CImuBrick_API& operator=(const CImuBrick_API&) = delete;
    CImuBrick_API& operator=(CImuBrick_API&&) = delete;

    void lock() { m_mutex.lock(); }
    const ego_data& getData() { return m_egoData; }
    void unlock() { m_mutex.unlock(); }

    bool setup(int sampleRate);
    void start();
    void suspend();
    void stop();

private:
    void cb_connect(const char *uid);

    static void cb_all_data(
        int16_t acceleration[3], int16_t magnetic_field[3],
        int16_t angular_velocity[3], int16_t euler_angle[3],
        int16_t quaternion[4], int16_t linear_acceleration[3],
        int16_t gravity_vector[3], int8_t temperature,
        uint8_t calibration_status, void* user_data);

    static void cb_enumerate(
        const char* uid,
        const char* connected_uid,
        char position,
        uint8_t hardware_version[3],
        uint8_t firmware_version[3],
        uint16_t device_identifier,
        uint8_t enumeration_type,
        void* user_data);

    IPConnection m_connection;
    IMUV2 m_imu;

    bool isConnected = false;

    int m_period;

    std::mutex m_mutex;
    ego_data m_egoData;
};

#endif // CImuBrick_API_API_H
