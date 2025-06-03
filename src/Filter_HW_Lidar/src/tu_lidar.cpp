// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CyC_TYPES.h"
#include <iostream>
#include "happly.h"
#include "CLivoxApi.h"

void showUsage()
{
    printf("\nUsage:\n"
        "tu_Lidar\n");
    exit(1);
}

int main(int argc, char ** argv)
{
    std::string sLidarFile = "../Nuscenes/Nuscenes_4/scene-0103/datastream_1_7/samples/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin.ply";
    
    happly::PLYData ply(sLidarFile);

    if (ply.getElementNames().size() > 0)
    {
        std::string elemName = ply.getElementNames()[0];
        std::vector<float> X = ply.getElement(elemName).getProperty<float>("x");
        std::vector<float> Y = ply.getElement(elemName).getProperty<float>("y");
        std::vector<float> Z = ply.getElement(elemName).getProperty<float>("z");

        //for (size_t i = 0; i < X.size(); ++i)
        //    std::cout << X[i] << ";  " << Y[i] << ";  " << Z[i] << std::endl;
    }

    return EXIT_SUCCESS;
}
