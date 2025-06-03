// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CMONO_CAMERA_RASPI_INTERFACE_H
#define CMONO_CAMERA_RASPI_INTERFACE_H

#include "CyC_TYPES.h"
#include "raspicam_cv.h"
#include <opencv2/opencv.hpp>

class CMonoCameraRaspiInterface
{
public:
    CMonoCameraRaspiInterface() :
        m_isInitialized(false)
    {
        
    }
    ~CMonoCameraRaspiInterface();

    bool openCamera();
    bool process();
    cv::Mat get_image() { return m_capturedImage; }

private:
    raspicam::RaspiCam_Cv m_raspiCamInstance;
    cv::Mat m_capturedImage;
    bool m_isInitialized;
};

#endif //CMONO_CAMERA_RASPI_INTERFACE_H
