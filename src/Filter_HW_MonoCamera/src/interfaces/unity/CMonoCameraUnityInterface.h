#ifndef CMONO_CAMERA_UNITY_INTERFACE_H
#define CMONO_CAMERA_UNITY_INTERFACE_H

#include "CyC_TYPES.h"
#include "enet/enet.h"

class CMonoCameraUnityInterface
{
public:
    CMonoCameraUnityInterface() :
        m_IpAddress(""),
        m_PortNumber(""),
        m_WaitTime(0),
        m_IsInitialized(false)
    {}
    ~CMonoCameraUnityInterface();

    bool enable(std::string ip, std::string port);
    bool process();
    cv::Mat get_image() { return m_Image; }

private:
    cv::Mat m_Image;
    std::string m_IpAddress;
    std::string m_PortNumber;
    ENetAddress m_Address;
    ENetHost* m_Client = nullptr;
    ENetPeer* m_Peer = nullptr;
    CyC_INT m_WaitTime;
    bool m_IsInitialized;

    bool EventHandler(ENetEvent event);
    bool ExtractImageData(enet_uint8* packet, size_t packetSize);

    cv::Mat ConvertStringToCvMat(const std::string& stringImage);
    std::string base64_decode(std::string const& encodedString);
    bool is_base64(unsigned char c);
};

#endif //CMONO_CAMERA_UNITY_INTERFACE_H
