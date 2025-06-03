// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu
 
#include "CMonoCameraRaspiInterface.h"

bool CMonoCameraRaspiInterface::openCamera()
{
    // Set camera params
    m_raspiCamInstance.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    m_raspiCamInstance.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    m_raspiCamInstance.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    //Open camera

    if (!m_raspiCamInstance.open()) 
    {
        spdlog::error("CMonoCameraRaspiInterface: Failed to open camera!");
        
        return false;
    }
    
    spdlog::info("CMonoCameraRaspiInterface: Camera has been opened!");
        
    m_isInitialized = true;
    return m_isInitialized;
}

bool CMonoCameraRaspiInterface::process()
{
    if (m_isInitialized)
    {
        m_raspiCamInstance.grab();
        m_raspiCamInstance.retrieve (m_capturedImage);
    }
    
    //spdlog::info("m_capturedImage has {} cols and {} rows", m_capturedImage.cols, m_capturedImage.rows);
    
    return true;
}

CMonoCameraRaspiInterface::~CMonoCameraRaspiInterface()
{
    m_raspiCamInstance.release();
}

