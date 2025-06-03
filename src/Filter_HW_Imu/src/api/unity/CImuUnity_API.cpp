// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImuUnity_API.h"
#include <json.hpp>

bool CImuUnity_API::enable(std::string ip, std::string port)
{
    m_IpAddress = ip;
    m_PortNumber = port;
    if (enet_initialize() != 0) {
        spdlog::error("CImuUnity_API: An error occurred while initializing ENet.");
    }

    enet_address_set_host(&m_Address, m_IpAddress.c_str());
    m_Address.port = (ushort)std::atoi(m_PortNumber.c_str());

    m_Client = enet_host_create(NULL, 32, 1, 0, 0);

    if (m_Client == NULL) {
        spdlog::error("CImuUnity_API: An error occured while initializing Imu Interface client.");
        return false;
    }

    m_Peer = enet_host_connect(m_Client, &m_Address, 5, 0);
    if (m_Peer == NULL) {
        spdlog::error("CImuUnity_API: No available peers for initiating an ENet connection");
        return false;
    }

    m_IsInitialized = true;
    return true;
}

CImuUnity_API::~CImuUnity_API()
{
    if (m_Client != nullptr)
        enet_host_destroy(m_Client);
}

bool CImuUnity_API::process()
{
    if (m_IsInitialized)
    {
        bool ret = false;
        ENetEvent event;
        if (enet_host_service(m_Client, &event, m_WaitTime) > 0) {
            ret = EventHandler(event);
        }

        enet_host_flush(m_Client);
        return ret;
    }
    return false;
}

bool CImuUnity_API::EventHandler(ENetEvent event)
{
    bool ret = false;

    switch (event.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
        spdlog::info("CImuUnity_API: Connected to {}:{}", event.peer->address.host, event.peer->address.port);
        break;
    case ENET_EVENT_TYPE_RECEIVE:
        ret = ExtractImuData(event.packet->data, event.packet->dataLength);
        /* Clean up the packet now that we're done using it. */
        enet_packet_destroy(event.packet);
        break;
    case ENET_EVENT_TYPE_DISCONNECT:
        spdlog::info("{} disconnected ", event.peer->data);
        /* Reset the peer's client information. */
        event.peer->data = NULL;
        break;
    case ENET_EVENT_TYPE_NONE:
        break;
    }

    return ret;
}

bool CImuUnity_API::ExtractImuData(enet_uint8* packet, size_t packetSize)
{
    bool ret = false;
    std::string packetAllData((char*)packet, packetSize);

    nlohmann::json dataJson = nlohmann::json::parse(packetAllData);

    // Get Imu Data
    if (dataJson["ImuData"].get<std::string>() != "")
    {
        std::vector<std::string> StringImuData = SplitString(
            SplitString(dataJson["ImuData"].get<std::string>(), "&")[0],
            ";");

        // StringImuData = [accX, accY, accZ, gyroX, gyroY, gyroZ, compass, roll, pitch, yaw]
        m_imu.acc.x() = std::stof(StringImuData[0]);
        m_imu.acc.y() = std::stof(StringImuData[1]);
        m_imu.acc.z() = std::stof(StringImuData[2]);
        m_imu.gyro.x() = std::stof(StringImuData[3]);
        m_imu.gyro.y() = std::stof(StringImuData[4]);
        m_imu.gyro.z() = std::stof(StringImuData[5]);
        m_imu.magnet.x() = std::stof(StringImuData[6]);
        m_imu.magnet.y() = std::stof(StringImuData[7]);
        m_imu.magnet.z() = std::stof(StringImuData[8]);
        m_imu.quat.update(std::stof(StringImuData[9]), 
            std::stof(StringImuData[10]), 
            std::stof(StringImuData[11]), 
            std::stof(StringImuData[12]));
        
        ret = true;
    }
    
    return ret;
}

std::vector<std::string> CImuUnity_API::SplitString(std::string str, const std::string delimiter)
{
    std::vector<std::string> result;
    size_t pos = 0;
    std::string token;

    while ((pos = str.find(delimiter)) != std::string::npos) {
        token = str.substr(0, pos);
        result.push_back(token);
        str.erase(0, pos + delimiter.length());
    }
    return result;
}
