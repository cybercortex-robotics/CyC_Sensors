// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/* 
    3D poCyC_INTs from depth:
    K = [[f, 0, Cu],
         [0, f, Cv],
         [0, 0, 1 ]]
    p2d = [u,v,1]
    P = [X,Y,Z]

    P = ( inv(K) * p2d ) * depth
*/
#include "CyC_TYPES.h"
#include <vector>
#include <iostream>
#include <filesystem>
#include <assert.h>
#include "CRgbdUtils.h"
#include "sensors/CPinholeCameraSensorModel.h"
//#include "COpenNI2API.h"
#include "CRealSense2Api.h"
#include "env/COcTreeUtils.h"
#include "vision/CDepthImageProcessing.h"
#include "vision/CImageProcessing.h"
#include "vision/CProjectiveGeometry.h"

//#include <QtWidgets/QApplication>
//#include <QtCore/QThread>
//#include <QtCore/QString>
//#include "ViewerGui.h"
//#include "CDrawingPrimitives.h"
//#include "CDrawerWorld.h"

bool        bSaveImages(false);
std::string sSaveFolder;

std::unique_ptr<CPinholeCameraSensorModel> pCamSensorModel;
CRgbdUtils::RGBDCameraApiType m_InterfaceType = CRgbdUtils::RGBD_CAMERA_OPENNI_API;
//std::unique_ptr<COpenNI2API> openni_api;
std::shared_ptr<CRealSense2Api> m_RealSense2 = CRealSense2Api::create_instance();

CTimer timer;
cv::Point ptMouseDepth(0, 0);

void onMouse(CyC_INT event, CyC_INT x, CyC_INT y, CyC_INT, void*);
void cam_thread();

void showUsage()
{
    printf("\nUsage:\n"
        "tu_RGBDCamera calibration_file.cal\n\n"
        "Options:\n"
        "  --oni    # OpenNI2 interface [default]\n"
        "  --rs     # RealSense2 interface\n"
        "  --save   # Save the RGB and Depth images to the specified folder\n"
        "eg: tu_RGBDCamera ../etc/calibration/modelcar/kinect_camera.cal\n");
    exit(1);
}

/*
class ViewingThread : public QThread
{
public:
    void setScene(octomap::ViewerGui* scene)
    {
        m_pDrawingScene = scene;

        // Add drawers
        m_pCamDrawer = new CDrawerPose("cam");
        m_pDrawingScene->addCycDrawer(m_pCamDrawer);

        m_pWorldDrawer = new CDrawerPose("World");
        m_pDrawingScene->addCycDrawer(m_pWorldDrawer);
    };

    void stop()
    {};

    void run()
    {
        Eigen::Matrix4f T_cam2world = CPose::Rt2T(-90.f * DEG2RAD, 0.f, -90.f * DEG2RAD, 2.f, 0.f, 5.f);

        // Images reading loop
        CyC_INT idx_start_image = 0;
        CyC_INT step = 1;
        for (CyC_INT k = idx_start_image;; k = k + step)
        {
            std::cout << "idx: " << k << std::endl;

            // Draw camera pose
            static_cast<CDrawerPose*>(m_pCamDrawer)->updatePose(T_cam2world);
            
            cv::Mat img_rgb, img_depth, img_depth_meter;
            if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
            {
                if (!openni_api->grab(img_rgb, img_depth))
                    continue;
                img_depth_meter = CDepthImageProcessing::asus2depth(img_depth);
            }
            else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
            {
                if (!CRealSense2API::instance().grab_images(img_rgb, img_depth_meter))
                    continue;
            }

            CcrOcTree* ocTree = new CcrOcTree(0.1f);
            COcTreeUtils::depth2octree(pCamSensorModel.get(), img_depth_meter, CPose(T_cam2world), ocTree, 1, 300.f, 1);

            m_pDrawingScene->openTreeInGUI(ocTree);
            
            if (!img_rgb.empty())
                cv::imshow("rgb", img_rgb);

            if (!img_depth.empty())
                cv::imshow("depth", img_depth);

            cv::waitKey(10);
        }
    };

private:
    octomap::ViewerGui* m_pDrawingScene;
    octomap::SceneObject* m_pWorldDrawer;
    octomap::SceneObject* m_pCamDrawer;
};
*/

CyC_INT main(CyC_INT argc, char** argv)
{
    // Do not use scientific notation
    std::cout << std::fixed;
    std::cout << std::setprecision(6);
    
    if (argc < 2)
    {
        showUsage();
        return EXIT_SUCCESS;
    }

    for (CyC_INT i = 1; i < argc - 1; i++)
    {
        if (strcmp(argv[i], "--oni") == 0)
        {
            m_InterfaceType = CRgbdUtils::RGBD_CAMERA_OPENNI_API;
            continue;
        }
        else if (strcmp(argv[i], "--rs") == 0)
        {
            m_InterfaceType = CRgbdUtils::RGBD_CAMERA_REALSENSE_API;
            continue;
        }
        else if (strcmp(argv[i], "--save") == 0)
        {
            bSaveImages = true;
            sSaveFolder = argv[i + 1];
            ++i;
            continue;
        }
    }

    // Check if folder exists
    if (bSaveImages && !CFileUtils::FolderExist(sSaveFolder.c_str()))
    {
        std::cout << "Storage folder not found. Exiting." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Load calibration data
    std::string sCalibrationFile = argv[argc - 1];
    if (!CFileUtils::FileExist(sCalibrationFile.c_str()))
    {
        std::cout << "ERROR: Camera calibration file not found. Exiting." << std::endl;
        return EXIT_FAILURE;
    }

    // Init camera
    pCamSensorModel = std::make_unique<CPinholeCameraSensorModel>(sCalibrationFile);

    if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
    {
        //openni_api = std::make_unique<COpenNI2API>(pCamSensorModel.get());
        //if (!openni_api->start())
        {
            std::cout << "ERROR: Could not start OpenNI2 camera. Exiting." << std::endl;
            return EXIT_FAILURE;
        }
        //else
        //{
        //    std::cout << "Started OpenNI2 camera." << std::endl;
        //}
    }
    else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
    {   
        if (!m_RealSense2->start())
        {
            std::cout << "ERROR: Could not start RealSense2 camera. Exiting." << std::endl;
            return EXIT_FAILURE;
        }
        else
        {
            std::cout << "Started RealSense2 camera." << std::endl;
        }
    }
    else
    {
        std::cout << "ERROR: RGBD camera interface undefined. Exiting." << std::endl;
        return EXIT_FAILURE;
    }

    /*QApplication pApp(argc, argv);
    octomap::ViewerGui m_drawingScene;
    m_drawingScene.show();
    ViewingThread mpThread;
    mpThread.setScene(&m_drawingScene);
    mpThread.run();
    pApp.exec();*/
    
    std::thread m_ImageThread = std::thread(&cam_thread);
    m_ImageThread.join();

    /*if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
        openni_api->stop();
    else*/ if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
    m_RealSense2->stop();

    return EXIT_SUCCESS;
}

void onMouse(CyC_INT event, CyC_INT x, CyC_INT y, CyC_INT, void*)
{
    ptMouseDepth.x = x;
    ptMouseDepth.y = y;
}

void cam_thread()
{
    CyC_INT idx = 0;
    char str[128];

    std::string sRgbFolder = sSaveFolder + "/left";
    std::string sDepthFolder = sSaveFolder + "/right";
    if (bSaveImages)
    {
        for (const auto& entry : std::filesystem::directory_iterator(sSaveFolder))
            std::filesystem::remove_all(entry.path());

        if (!std::filesystem::create_directory(sRgbFolder) || !std::filesystem::create_directory(sDepthFolder))
        {
            std::cout << "ERROR: Storage subfolders could not be created. Exiting." << std::endl;
            return;
        }
    }

    cv::namedWindow("depth");

    while (true)
    {
        timer.restart();
        CyC_TIME_UNIT ts;
        cv::Mat img_rgb, img_depth_mm;
        CycImus imu_cache;

        /*if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
            openni_api->grab(img_rgb, img_depth_mm);
        else*/ if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
            m_RealSense2->get_rs_data(ts, img_rgb, img_depth_mm, imu_cache);

        if (!img_rgb.empty() && !img_depth_mm.empty())
        {
            if (bSaveImages)
            {
                snprintf(str, sizeof(str) - 1, "%s/%06d.png", sRgbFolder.c_str(), idx);
                //cv::imwrite(str, rgb.clone());

                //std::this_thread::sleep_for(std::chrono::milliseconds(10));

                snprintf(str, sizeof(str) - 1, "%s/%06d.png", sDepthFolder.c_str(), idx);
                //cv::imwrite(str, depth.clone());

                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (!img_depth_mm.empty())
            {
                cv::Mat img_depth_meter;

                if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
                    img_depth_meter = CDepthImageProcessing::asus2depth(img_depth_mm);
                else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
                    img_depth_meter = img_depth_mm.clone();

                cv::setMouseCallback("depth", onMouse, 0);
                float depth_meters = img_depth_meter.at<float>(ptMouseDepth.y, ptMouseDepth.x);

                float fDepthScaleFactor = 1000.f / 16.f;  // Factor used to convert from meters to depth in range of [0-255]
                cv::Mat imgR8UC1;
                img_depth_meter.convertTo(imgR8UC1, CV_8UC1, fDepthScaleFactor);

                snprintf(str, sizeof(str) - 1, "(%d, %d): %f", ptMouseDepth.x, ptMouseDepth.y, depth_meters);
                cv::putText(imgR8UC1, str, cv::Point(15, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, color::white, 1);

                cv::imshow("depth", imgR8UC1);
            }

            timer.stop();
            std::cout << "Time: " << timer.getElapsedTimeMilliseconds() << "\tIMU samples: " << imu_cache.size() << std::endl;

            cv::imshow("rgb", img_rgb);
            cv::waitKey(30);
        }

        idx++;
    }
}
