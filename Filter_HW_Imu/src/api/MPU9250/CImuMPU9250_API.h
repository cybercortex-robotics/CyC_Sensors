// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CImuMPU9250_API_H
#define CImuMPU9250_API_H

#include "CyC_TYPES.h"
#include "rs232.h"

class CImuMPU9250_API
{
public:
    CImuMPU9250_API() = default;
    CImuMPU9250_API(const CImuMPU9250_API&) = default;
    CImuMPU9250_API(CImuMPU9250_API&&) = default;
    CImuMPU9250_API& operator=(const CImuMPU9250_API&) = default;
    CImuMPU9250_API& operator=(CImuMPU9250_API&&) = default;
    ~CImuMPU9250_API() = default;

    bool    start(const CyC_INT& _port_nr);
    void        stop();
    bool    isRunning();
    bool    read(CycImu& _out_imu);

private:
    std::string                      readSerialLine();
    static std::vector<std::string>  splitLine(const std::string& line);

private:
    CyC_INT     m_PortNr = -1;
    bool    m_bRunning = false;
};
#endif // CImuMPU9250_API_H