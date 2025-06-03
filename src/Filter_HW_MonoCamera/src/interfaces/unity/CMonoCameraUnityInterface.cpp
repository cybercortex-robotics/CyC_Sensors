#include "CMonoCameraUnityInterface.h"
#include <json.hpp>
#include <opencv2/opencv.hpp>

bool CMonoCameraUnityInterface::enable(std::string ip, std::string port)
{
    m_IpAddress = ip;
    m_PortNumber = port;
    if (enet_initialize() != 0) {
        spdlog::error("CMonoCameraUnityInterface: An error occurred while initializing ENet.");
    }

    enet_address_set_host(&m_Address, m_IpAddress.c_str());
    m_Address.port = (ushort)std::atoi(m_PortNumber.c_str());

    m_Client = enet_host_create(NULL, 32, 1, 0, 0);

    if (m_Client == NULL) {
        spdlog::error("CMonoCameraUnityInterface: An error occured while initializing monocamera interface client");
        return false;
    }

    m_Peer = enet_host_connect(m_Client, &m_Address, 5, 0);
    if (m_Peer == NULL) {
        spdlog::error("CMonoCameraUnityInterface: No available peers for initiating an ENet connection");
        return false;
    }

    m_IsInitialized = true;
    return true;
}

bool CMonoCameraUnityInterface::process()
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

bool CMonoCameraUnityInterface::EventHandler(ENetEvent event)
{
    bool ret = false;
    switch (event.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
        spdlog::info("CMonoCameraUnityInterface: Connected to {}:{}", event.peer->address.host, event.peer->address.port);
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

bool CMonoCameraUnityInterface::ExtractImageData(enet_uint8* packet, size_t packetSize)
{
    bool ret = false;
    std::string packetAllData((char*)packet, packetSize);

    nlohmann::json dataJson = nlohmann::json::parse(packetAllData);

    if (dataJson["RGB"].get<std::string>() != "")
    {
        m_Image = ConvertStringToCvMat(dataJson["RGB"].get<std::string>());
        ret = true;
        
    }
    return ret;
}

CMonoCameraUnityInterface::~CMonoCameraUnityInterface()
{
    if (m_Client != nullptr)
        enet_host_destroy(m_Client);
}


cv::Mat CMonoCameraUnityInterface::ConvertStringToCvMat(const std::string& stringImage)
{
    std::string decodedString = base64_decode(stringImage);
    std::vector<uchar> data(decodedString.begin(), decodedString.end());

    cv::Mat imgMat = cv::imdecode(data, cv::IMREAD_UNCHANGED);
    return imgMat;
}

std::string CMonoCameraUnityInterface::base64_decode(std::string const& encodedString) 
{
    const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    size_t in_len = encodedString.size();
    CyC_INT i = 0;
    CyC_INT j = 0;
    CyC_INT in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::string ret;

    while (in_len-- && (encodedString[in_] != '=') && is_base64(encodedString[in_])) {
        char_array_4[i++] = encodedString[in_]; in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++)
                char_array_4[i] = (unsigned char)base64_chars.find(char_array_4[i]);

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
            char_array_4[j] = (unsigned char)base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
    }

    return ret;
}

bool CMonoCameraUnityInterface::is_base64(unsigned char c) 
{
    return (isalnum(c) || (c == '+') || (c == '/'));
}
