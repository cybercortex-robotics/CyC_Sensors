// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CRgbdCameraUnityApi.h"
#include <json.hpp>
#include <opencv2/opencv.hpp>

bool CRGBDCameraUnityApi::enable(std::string ip, std::string port)
{
    m_IpAddress = ip;
    m_PortNumber = port;
    if (enet_initialize() != 0) {
        spdlog::error("CRGBDCameraUnityInterface: An error occurred while initializing ENet.");
    }

    enet_address_set_host(&m_Address, m_IpAddress.c_str());
    m_Address.port = (ushort)std::atoi(m_PortNumber.c_str());

    m_Client = enet_host_create(NULL, 32, 1, 0, 0);

    if (m_Client == NULL) {
        spdlog::error("CRGBDCameraUnityInterface: An error occured while initializing rgbd camera interface client");
        return false;
    }

    m_Peer = enet_host_connect(m_Client, &m_Address, 5, 0);
    if (m_Peer == NULL) {
        spdlog::error("CRGBDCameraUnityInterface: No available peers for initiating an ENet connection");
        return false;
    }

    m_IsInitialized = true;
    return true;
}

bool CRGBDCameraUnityApi::process()
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

bool CRGBDCameraUnityApi::EventHandler(ENetEvent event)
{
    bool ret = false;
    switch (event.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
        spdlog::info("CRGBDCameraUnityInterface: Connected to {}:{}", event.peer->address.host, event.peer->address.port);
        break;
    case ENET_EVENT_TYPE_RECEIVE:
        ret = ExtractImageData(event.packet->data, event.packet->dataLength);
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

bool CRGBDCameraUnityApi::ExtractImageData(enet_uint8* packet, size_t packetSize)
{
    bool ret = false;
    std::string packetAllData((char*)packet, packetSize);

    nlohmann::json dataJson = nlohmann::json::parse(packetAllData);

    // Get Raw image
    if (dataJson["RGB"].get<std::string>() != "" && dataJson["Depth"].get<std::string>() != "")
    {
        m_ImgRGB = ConvertStringToCvMat(dataJson["RGB"].get<std::string>());
        
        cv::Mat enc_depth = ConvertStringToCvMat(dataJson["Depth"].get<std::string>());
        m_ImgDepth = cv::Mat(enc_depth.rows, enc_depth.cols, CV_32F);
        for (CyC_INT i = 0; i < m_ImgDepth.rows; ++i)
        {
            for (CyC_INT j = 0; j < m_ImgDepth.cols; ++j)
            {
                cv::Vec3b px = enc_depth.at<cv::Vec3b>(i, j);
                m_ImgDepth.at<float>(i, j) = 1000.f * ((float)px[2] + (float)px[1] * 256.f + (float)px[0] * 65536.f) / (16777215.f);
            }
        }

        ret = true;
    }

    // Get gray depth image
    /*
    if (!m_Raw.empty())
    {
        m_Depth = ConvertRawToGrayDepth(m_Raw);
        ret = true;
    }
    */

    //if (dataJson["ImagesDataDepth"].get<std::string>() != "")
    {
        //spdlog::info("+++++++");
        //m_ImgDepth = ConvertStringToCvMat(dataJson["ImagesDataDepth"].get<std::string>());
        //ret = true;
    }

    return ret;
}

CRGBDCameraUnityApi::~CRGBDCameraUnityApi()
{
    if (m_Client != nullptr)
        enet_host_destroy(m_Client);
}


cv::Mat CRGBDCameraUnityApi::ConvertStringToCvMat(const std::string& stringImage)
{
    std::string decodedString = base64_decode(stringImage);
    std::vector<uchar> data(decodedString.begin(), decodedString.end());

    cv::Mat imgMat = cv::imdecode(data, cv::IMREAD_UNCHANGED);
    return imgMat;
}

cv::Mat CRGBDCameraUnityApi::ConvertRawToGrayDepth(cv::Mat raw_image)
{
    // NOT WORKING PROPERLY
    /* Notes
      From https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera
      normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
      in_meters = 1000 * normalized

      raw_image.type() -> 16 = CV_8UC3
    */

    // Convert to float and split into channels
    cv::Mat raw_image_float;
    raw_image.convertTo(raw_image_float, CV_32FC3);
    std::vector<cv::Mat> raw_ch(3);
    cv::split(raw_image_float / 256.0, raw_ch); // B, G, R

    // Create and calculate 1 channel gray image of depth based on the formula
    cv::Mat depth_gray = cv::Mat::zeros(raw_ch[0].rows, raw_ch[0].cols, CV_32FC1);
    depth_gray = 1000.0 * (raw_ch[2] + raw_ch[1] * 256.0 + raw_ch[0] * 65536.0) / (16777215.0);

    // Create final BGR image
    //cv::Mat depth_3d;
    //cv::cvtColor(depth_gray, depth_3d, cv::COLOR_GRAY2BGR);

    return depth_gray;
}

std::string CRGBDCameraUnityApi::base64_decode(std::string const& encodedString) 
{
    const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    size_t in_len = encodedString.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::string ret;

    while (in_len-- && (encodedString[in_] != '=') && is_base64(encodedString[in_])) {
        char_array_4[i++] = encodedString[in_]; in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                ret += char_array_3[i];
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 4; j++)
            char_array_4[j] = 0;

        for (j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
    }

    return ret;
}

bool CRGBDCameraUnityApi::is_base64(unsigned char c) 
{
    return (isalnum(c) || (c == '+') || (c == '/'));
}
