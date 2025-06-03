// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImuMPU9250_API.h"

bool CImuMPU9250_API::start(const CyC_INT& _port_nr)
{
    m_PortNr = _port_nr;
    const char mode[] = { '8', 'N', '1', 0 };
    
    m_bRunning = !RS232_OpenComport(m_PortNr, 115200, mode, 0);
    
    return m_bRunning;
}

void CImuMPU9250_API::stop()
{
    RS232_CloseComport(m_PortNr);
}

bool CImuMPU9250_API::isRunning()
{
    return m_bRunning;
}

bool CImuMPU9250_API::read(CycImu& _out_imu)
{
    bool bReturn = false;

    std::string line = readSerialLine();
    std::vector<std::string> vec = splitLine(line);
    auto all_numbers = [](std::string val) {return isdigit(val[0]) || (val.size() > 2 && val[0] == '-' && isdigit(val[1])); };

    if (vec.size() == 10 && std::all_of(std::begin(vec), std::end(vec), all_numbers)) // prevent stof ERANGE exception
    {
        float deltat = std::stof(vec[0]);
        _out_imu.acc.x() = std::stof(vec[1]);
        _out_imu.acc.y() = std::stof(vec[2]);
        _out_imu.acc.z() = std::stof(vec[3]);
        _out_imu.gyro.x() = std::stof(vec[4]);
        _out_imu.gyro.y() = std::stof(vec[5]);
        _out_imu.gyro.z() = std::stof(vec[6]);
        _out_imu.magnet.x() = std::stof(vec[7]);
        _out_imu.magnet.y() = std::stof(vec[8]);
        _out_imu.magnet.z() = std::stof(vec[9]);

        bReturn = true;
    }

    return bReturn;
}

std::string CImuMPU9250_API::readSerialLine()
{
    unsigned char ch = 0;
    std::string line;

    while (ch != 10)
        if (RS232_PollComport(m_PortNr, &ch, 1))
            line += ch;

    return line;
}

std::vector<std::string> CImuMPU9250_API::splitLine(const std::string& line)
{
    std::stringstream sstream(line);
    std::vector<std::string> vec;
    std::string temp;
    while (getline(sstream, temp, ' '))
        vec.push_back(temp);
    return vec;
}
