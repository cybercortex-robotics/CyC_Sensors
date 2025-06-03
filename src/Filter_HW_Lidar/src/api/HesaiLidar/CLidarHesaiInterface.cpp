// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CLidarHesaiInterface.h"
#include "CyC_TYPES.h"

void CHesaiLidarInterface::gpsCallback(int timestamp) {
  //printf("gps from callback: %d\n", timestamp);
}

void CHesaiLidarInterface::lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
  //spdlog::info("Received LIDAR Point Cloud, timestamp: {}, cloud size: {}", timestamp, cld->points.size());

  m_lidarOutputVoxels.clear();

  for(CyC_UINT i = 0; i < cld->points.size(); i++)
  {
      CycVoxel voxel(cld->points.at(i).x, cld->points.at(i).y, cld->points.at(i).z);
      m_lidarOutputVoxels.emplace_back(voxel);
  }

  //spdlog::info("m_lidarOutputVoxels size from lidarCallback: {}", m_lidarOutputVoxels.size());
}

void CHesaiLidarInterface::lidarAlgorithmCallback(HS_Object3D_Object_List* object_t) {
    HS_Object3D_Object* object;
    printf("----------------------\n");
    printf("total objects: %d\n",object_t->valid_size);
    for (size_t i = 0; i < object_t->valid_size; i++) {
        object = &object_t->data[i];
        printf("id: %u, type: %u\n",object->data.id, object->type);
    }
    printf("----------------------\n");
}

bool CHesaiLidarInterface::enable()
{
    //PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 0, 10110, \
    //    lidarCallback, lidarAlgorithmCallback, gpsCallback, 0, 0, 0, std::string("Pandar40P"), std::string("frame_id"),"");

    /*std::string filePath = "/home/xavier/Lidar_pcap/correction_file_Pandar40.csv";
    std::ifstream fin(filePath);
    int length = 0;
    std::string strlidarCalibration;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    strlidarCalibration = buffer;

    m_pandarGeneral->LoadLidarCorrectionFile(strlidarCalibration);*/

    m_pandarGeneral->Start();

    return true;
}

void CHesaiLidarInterface::getLidarOutputVoxels(CycVoxels& _lidar_voxels)
{
    _lidar_voxels = m_lidarOutputVoxels;
}
