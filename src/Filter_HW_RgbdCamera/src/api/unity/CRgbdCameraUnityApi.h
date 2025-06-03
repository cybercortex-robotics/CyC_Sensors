// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CRGBD_CAMERA_UNITY_INTERFACE_H
#define CRGBD_CAMERA_UNITY_INTERFACE_H

#include "CyC_TYPES.h"
#include "enet/enet.h"

class CRGBDCameraUnityApi
{
public:
    CRGBDCameraUnityApi() :
        m_IpAddress(""),
        m_PortNumber(""),
        m_WaitTime(0),
        m_IsInitialized(false)
    {}
    ~CRGBDCameraUnityApi();

    bool enable(std::string ip, std::string port);
    bool process();
    cv::Mat get_rgb() { return m_ImgRGB; }
    cv::Mat get_depth() { return m_ImgDepth; }

private:
    cv::Mat m_ImgRGB; 
    cv::Mat m_ImgDepth;
    std::string m_IpAddress;
    std::string m_PortNumber;
    ENetAddress m_Address;
    ENetHost* m_Client = nullptr;
    ENetPeer* m_Peer = nullptr;
    CyC_INT m_WaitTime;
    bool m_IsInitialized = false;

    bool EventHandler(ENetEvent event);
    bool ExtractImageData(enet_uint8* packet, size_t packetSize);

    cv::Mat ConvertStringToCvMat(const std::string& stringImage);
    cv::Mat ConvertRawToGrayDepth(cv::Mat raw_image);
    std::string base64_decode(std::string const& encodedString);
    bool is_base64(unsigned char c);
};

#endif //CRGBD_CAMERA_UNITY_INTERFACE_H
