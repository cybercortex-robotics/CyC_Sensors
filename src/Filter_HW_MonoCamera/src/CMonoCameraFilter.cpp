// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CMonoCameraFilter.h"
#include "vision/CImageProcessing.h"
#include "vision/CDepthImageProcessing.h"
#ifdef ROBOT_ARPAROTDRONE_ENABLED
#include <CARParotDrone.h>
#endif // ROBOT_ARPAROTDRONE_ENABLED
#ifdef ROBOT_PARROTANAFI_ENABLED
#include <CParrotANAFI.h>
#endif // ROBOT_PARROTANAFI_ENABLED

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CMonoCameraFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CMonoCameraFilter(_params);
}

CMonoCameraFilter::CMonoCameraFilter(CycDatablockKey _key) :
    CCycFilterBase(_key),
    m_nDeviceID(0),
    m_bRectifyImages(false),
    m_nApiID(cv::CAP_ANY)	// 0 = autodetect default API
{
    setFilterType("CyC_MONO_CAMERA_FILTER_TYPE");

    // Assign the output data type
    m_OutputDataType = CyC_IMAGE;
    m_InputDataType = CyC_IMAGE;
}

CMonoCameraFilter::CMonoCameraFilter(ConfigFilterParameters _params) :
    CCycFilterBase(_params),
    m_nDeviceID(0),
    m_bRectifyImages(false),
    m_nApiID(cv::CAP_ANY)	// 0 = autodetect default API
{
    setFilterType("CyC_MONO_CAMERA_FILTER_TYPE");

    // Assign the output data type
    m_OutputDataType = CyC_IMAGE;
    m_InputDataType = CyC_IMAGE;

    // Load the sensor model
    if (!m_CustomParameters.at("CalibrationFile").empty())
    {
        CStringUtils::stringToBool(m_CustomParameters["Rectify"], m_bRectifyImages);
        fs::path calib_file = fs::path(_params.sGlobalBasePath) / fs::path(m_CustomParameters.at("CalibrationFile"));
        this->m_pSensorModel = new CPinholeCameraSensorModel(calib_file.string());
    }
    else
    {
        spdlog::error("Filter [{}-{}]: {}: Calibration file does not exist. Disabling filter.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
    }
}

CMonoCameraFilter::~CMonoCameraFilter()
{
    if (m_bIsEnabled)
        disable();

    delete this->m_pSensorModel;

#ifdef __ANDROID_API__
    delete m_ndkCamera;
    m_ndkCamera = NULL;
#endif
}

bool CMonoCameraFilter::enable()
{
    if (!isNetworkFilter())
    {
        // Get the pose data filter
        m_pPoseDataFilter = CFilterUtils::getStateFilter(this->getInputSources());
        
        // Read the voxels file
        if (!m_CustomParameters["voxels"].empty())
        {
            std::string voxels_path = m_CustomParameters.at("voxels");
            if (!CDepthImageProcessing::readVoxelsFile(voxels_path, m_Voxels))
                return false;
        }
        
        if (!isReplayFilter())
        {
#ifdef __ANDROID_API__
            m_ndkCamera = new NDKCamera();
#else
            bool ret = false;
            if (m_CustomParameters.find("interface") == m_CustomParameters.end()) 
            {
                spdlog::error("Filter [{}-{}]: CMonoCamera: Interface parameter not found in configuration file ", getFilterKey().nCoreID, getFilterKey().nFilterID);
                m_bIsEnabled = false;
                return false;
            }
            m_InterfaceType = StringToEnumType(m_CustomParameters["interface"]);

            if (m_InterfaceType == MONO_CAMERA_UNITY_INTERFACE) 
            {
                if (m_CustomParameters.find("ip") == m_CustomParameters.end() || m_CustomParameters.find("port") == m_CustomParameters.end())
                {
                    spdlog::error("Filter [{}-{}]: CMonoCamera: ip and/or port parameters not found in configuration file", getFilterKey().nCoreID, getFilterKey().nFilterID);
                    return false;
                }
#if !defined(RASPBERRY_PI)
                ret = m_UnityInterface.enable(m_CustomParameters["ip"], m_CustomParameters["port"]);
#endif
            }
            else if (m_InterfaceType == MONO_CAMERA_RASPI_INTERFACE) 
            {
#ifdef RASPBERRY_PI
                spdlog::info("Filter [{}-{}]: Opening the dedicated Raspberry Pi camera stream ...", getFilterKey().nCoreID, getFilterKey().nFilterID);
                ret = m_RaspiInterface.openCamera();
#endif
            }
            else if (m_InterfaceType == MONO_CAMERA_SIM_INTERFACE) 
            {
                spdlog::info("Filter [{}-{}]: Simultated camera image", getFilterKey().nCoreID, getFilterKey().nFilterID);
            }
            else if (m_InterfaceType == MONO_CAMERA_ARDRONE_INTERFACE)
            {
#ifdef ROBOT_ARPAROTDRONE_ENABLED
                if (!CARParotDrone::instance().open())
                {
                    spdlog::info("Filter [{}-{}]: ArDrone camera not available", getFilterKey().nCoreID, getFilterKey().nFilterID);
                    return false;
                }
#endif // ROBOT_ARPAROTDRONE_ENABLED
            }
            else if (m_InterfaceType == MONO_CAMERA_ANAFI_INTERFACE)
            {
#ifdef ROBOT_PARROTANAFI_ENABLED
                if (!CParrotANAFI::instance().isOpen() && !CParrotANAFI::instance().open(DEFAULT_ANAFI_UDP_IP, DEFAULT_ANAFI_UDP_PORT))
                {
                    spdlog::info("Filter [{}-{}]: ANAFI camera not available", getFilterKey().nCoreID, getFilterKey().nFilterID);
                    return false;
                }

                const auto systems = CParrotANAFI::instance().getDiscoveredSystems();
                CParrotANAFI::instance().getSystem(*systems.begin())->startCamera();
#endif // ROBOT_PARROTANAFI_ENABLED
            }
            else 
            {
                spdlog::info("Filter [{}-{}]: Opening the USB camera stream ...", getFilterKey().nCoreID, getFilterKey().nFilterID);

                // Open selected camera using selected API
                m_VideoCapture.open(m_nDeviceID + m_nApiID, cv::CAP_DSHOW);

                if (m_VideoCapture.isOpened())
                {
                    // Retrieve a frame for initializing the filter's output image
                    cv::Mat dummy_img;
                    m_VideoCapture.read(dummy_img);
                    
                    // check if we succeeded
                    if (dummy_img.empty())
                    {
                        spdlog::error("Filter [{}-{}]: Blank frame grabbed", getFilterKey().nCoreID, getFilterKey().nFilterID);
                        return false;
                    }
                }
            }
#endif
        }

        m_bIsEnabled = true;
        spdlog::info("Filter [{}-{}]: CMonoCamera::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID);
    }

    m_bIsEnabled = true;

    return true;
}

bool CMonoCameraFilter::disable()
{
    //if (m_InterfaceType == MONO_CAMERA_ARDRONE_INTERFACE)
    //    CARParotDrone::instance().close();

    if (isRunning())
        stop();

#ifdef __ANDROID_API__
        if (!isNetworkFilter())
        {
            delete m_ndkCamera;
        }
#else
    if (!isNetworkFilter() && m_VideoCapture.isOpened())
        m_VideoCapture.release();
#endif

    m_bIsEnabled = false;
    return true;
}

bool CMonoCameraFilter::process()
{
    // Update camera pose
    CPose pose;
    if (CFilterUtils::getPose(m_pPoseDataFilter, m_lastReadTsPose, pose))
        this->m_pSensorModel->updatePose(pose * this->m_pSensorModel->extrinsics());

#ifndef __ANDROID_API__
    cv::Mat frame;

    if (m_InterfaceType == MONO_CAMERA_SIM_INTERFACE)
    {
        CImageProcessing::simulateImg(static_cast<const CPinholeCameraSensorModel*>(this->getSensorModel()), m_Voxels, frame);
        
        // Simulate camera sampling time of 40ms
        std::this_thread::sleep_for(std::chrono::milliseconds(this->getDt()));
    }
    else if (m_InterfaceType == MONO_CAMERA_UNITY_INTERFACE)
    {
#if !defined(RASPBERRY_PI)
        while (!m_UnityInterface.process())
        {
            if (!isRunning())
            {
                return true;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        frame = m_UnityInterface.get_image();
#endif
    }
    else if (m_InterfaceType == MONO_CAMERA_RASPI_INTERFACE)
    {
#ifdef RASPBERRY_PI
        while (!m_RaspiInterface.process())
        {
            if (!isRunning())
            {
                return true;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        frame = m_RaspiInterface.get_image();
        //spdlog::info("Frame has {} cols and {} rows", frame.cols, frame.rows);
#endif
    }
    else if (m_InterfaceType == MONO_CAMERA_ARDRONE_INTERFACE)
    {
#ifdef ROBOT_ARPAROTDRONE_ENABLED
        // ARDrone image acquisition
        auto ardrone_frame = CARParotDrone::instance().getImage();
        frame = ardrone_frame.m_Image;
#endif // ROBOT_ARPAROTDRONE_ENABLED
    }
    else if (m_InterfaceType == MONO_CAMERA_ANAFI_INTERFACE)
    {
#ifdef ROBOT_PARROTANAFI_ENABLED
        if (CParrotANAFI::instance().isOpen())
        {
            auto systems = CParrotANAFI::instance().getDiscoveredSystems();
            frame = CParrotANAFI::instance().getSystem(*systems.begin())->getCameraImage();
        }
#endif // ROBOT_PARROTANAFI_ENABLED
    }
    else
    {
        // wait for a new frame from camera and store it into 'frame'
        m_VideoCapture.read(frame);
    }
#else

    while ((m_androidFrameId <= m_lastProcessedAndroidFrameId))
    {
        if (!isRunning())
        {
            return true;
        }

        m_androidFrameId = m_ndkCamera->getCurrentFrameID();

        //spdlog::info("CMonoCamera::process(): current m_androidFrameId {}", m_androidFrameId);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if ((m_androidFrameId > m_lastProcessedAndroidFrameId))
    {
        m_lastProcessedAndroidFrameId = m_androidFrameId;
    }
    else
    {
        spdlog::info("CMonoCamera::process(): else true");
        return true;
    }

    // Get the android frame from the camera running on another thread
    cv::Mat frame(m_ndkCamera->getCurrentFrame());
#endif

    // check if we succeeded
    if (frame.empty() && (m_InterfaceType != MONO_CAMERA_ANAFI_INTERFACE))
    {
        spdlog::error("Filter [{}-{}]: ERROR! blank frame grabbed", getFilterKey().nCoreID, getFilterKey().nFilterID);
        return false;
    }
    
    // Rectify image
    if (m_bRectifyImages)
        dynamic_cast<CPinholeCameraSensorModel*>(this->m_pSensorModel)->undistortImage(frame, frame);
    
    CycImages dst;
    dst.emplace_back(frame);
    updateData(dst);

    return true;
}

void CMonoCameraFilter::loadFromDatastream(const std::string& _datastream_entry, const std::string& _db_root_path)
{
#ifndef __ANDROID_API__
    this->m_bIsProcessing = true;
    CPinholeCameraSensorModel* sensor_model = dynamic_cast<CPinholeCameraSensorModel*>(this->m_pSensorModel);

    // Update sensor pose
    CPose pose;
    if (CFilterUtils::getPose(m_pPoseDataFilter, m_lastReadTsPose, pose))
        this->m_pSensorModel->updatePose(pose * this->m_pSensorModel->extrinsics());

    csv::reader::row row;
    row.parse_line(_datastream_entry, ',');

    enum { TS_STOP, SAMPLING_TIME, IMAGE_PATH_LEFT, IMAGE_PATH_RIGHT, NUM };
    if (row.size() != NUM)
    {
        spdlog::error("CMonoCamera: Wrong number of columns. {} provided, but expected {}.", row.size(), NUM);
        return;
    }

    const std::string img_path = _db_root_path + row.get<std::string>(IMAGE_PATH_LEFT);
    cv::Mat frame = cv::imread(img_path, cv::IMREAD_ANYCOLOR);
    cv::resize(frame, frame, cv::Size(sensor_model->width(), sensor_model->height()));

    if (m_bRectifyImages)
        sensor_model->undistortImage(frame, frame);

    CycImages output;
    output.emplace_back(frame);

    const auto tTimestampStop  = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime   = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;

    updateData(output, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);

    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    this->m_bIsProcessing = false;
#endif
}

int CMonoCameraFilter::StringToEnumType(const std::string& str_type)
{
    if (str_type.compare("sim") == 0)
        return MONO_CAMERA_SIM_INTERFACE;
    else if (str_type.compare("usb") == 0)
        return MONO_CAMERA_USB_INTERFACE;
    else if (str_type.compare("unity") == 0)
        return MONO_CAMERA_UNITY_INTERFACE;
    else if (str_type.compare("raspi") == 0)
        return MONO_CAMERA_RASPI_INTERFACE;
    else if (str_type.compare("ardrone") == 0)
        return MONO_CAMERA_ARDRONE_INTERFACE;
    else if (str_type == "anafi")
        return MONO_CAMERA_ANAFI_INTERFACE;

    return MONO_CAMERA_SIM_INTERFACE;
}
