// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CHESAI_LIDAR_INTERFACE_H
#define CHESAI_LIDAR_INTERFACE_H

#include "CyC_TYPES.h"
#include "include/pandarGeneral_sdk.h"

class CHesaiLidarInterface
{
public:
    CHesaiLidarInterface()
    {
        m_pandarGeneral = new PandarGeneralSDK(std::string("192.168.1.201"), 2368, 0, 10110,
        boost::bind(&CHesaiLidarInterface::lidarCallback, this, boost::placeholders::_1, boost::placeholders::_2),
        boost::bind(&CHesaiLidarInterface::lidarAlgorithmCallback, this, boost::placeholders::_1),
        boost::bind(&CHesaiLidarInterface::gpsCallback, this, boost::placeholders::_1),
        0, 0, 0, std::string("Pandar40P"), std::string("frame_id"),"");

        spdlog::info("PandarGeneralSDK initialized");

        m_isInitialized = true;
    }

    ~CHesaiLidarInterface();

    bool enable();
    void getLidarOutputVoxels(CycVoxels& _lidar_voxels);
    void gpsCallback(int timestamp);
    void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp);
    void lidarAlgorithmCallback(HS_Object3D_Object_List* object_t);

private:
    bool m_isInitialized;
    CycVoxels m_lidarOutputVoxels;
    PandarGeneralSDK* m_pandarGeneral;
};

#endif //CHESAI_LIDAR_INTERFACE_H
