// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CIMU_UNITY_INTERFACE_H
#define CIMU_UNITY_INTERFACE_H
#include "CyC_TYPES.h"
#include "enet/enet.h"

class CImuUnity_API
{
public:
    CImuUnity_API() :
        m_IpAddress(""),
        m_PortNumber(""),
        m_WaitTime(0),
        m_IsInitialized(false)
    {}
    ~CImuUnity_API();

    bool enable(std::string ip, std::string port);
    bool process();
    CycImu get_imu() { return m_imu; }

private:
    CycImu m_imu;
    std::string m_IpAddress;
    std::string m_PortNumber;
    ENetAddress m_Address;
    ENetHost* m_Client = nullptr;
    ENetPeer* m_Peer = nullptr;
    CyC_INT m_WaitTime;
    bool m_IsInitialized;

    bool EventHandler(ENetEvent event);
    bool ExtractImuData(enet_uint8* packet, size_t packetSize);
    std::vector<std::string> SplitString(std::string str, const std::string delimiter);
};
#endif // CIMU_UNITY_INTERFACE_H