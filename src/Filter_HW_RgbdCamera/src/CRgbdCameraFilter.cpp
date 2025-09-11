// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CRgbdCameraFilter.h"
#include <sensors/CPinholeCameraSensorModel.h>
#include <vision/CDepthImageProcessing.h>
#include <os/CFilterUtils.h>
#include <os/CCsvReader.h>

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CRgbdCameraFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CRgbdCameraFilter(_params);
}

CRgbdCameraFilter::CRgbdCameraFilter(CycDatablockKey _key) :
    CCycFilterBase(_key)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_RGBDCAMERA_FILTER_TYPE");
    m_OutputDataType = CyC_IMAGE;
}

CRgbdCameraFilter::CRgbdCameraFilter(const ConfigFilterParameters& _params) :
    CCycFilterBase(_params)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_RGBDCAMERA_FILTER_TYPE");
    m_OutputDataType = CyC_IMAGE;

    // Check if a realsense robot is already registered
    if (_params.pSingletonRegistry != nullptr)
    {
        if (_params.pSingletonRegistry->get<CRealSense2Api>() == nullptr)
            _params.pSingletonRegistry->registerInstance<CRealSense2Api>();
        m_RealSense = _params.pSingletonRegistry->get<CRealSense2Api>();
    }

    // Load the sensor model
    if (!_params.CustomParameters.at("CalibrationFile").empty())
    {
        CStringUtils::stringToBool(m_CustomParameters["Rectify"], m_bRectifyImages);
        fs::path calib_file = fs::path(_params.sGlobalBasePath) / fs::path(_params.CustomParameters.at("CalibrationFile"));
        this->m_pSensorModel = new CPinholeCameraSensorModel(calib_file.string());
    }
}

CRgbdCameraFilter::~CRgbdCameraFilter()
{
	if (m_bIsEnabled)
		disable();
}

bool CRgbdCameraFilter::enable()
{
    // Get the pose data filter
    if (!isNetworkFilter())
        m_pPoseDataFilter = CFilterUtils::getStateFilter(this->getInputSources());

    if (!isReplayFilter() && !isNetworkFilter())
    {
        // Get interface type
        if (m_CustomParameters.find("interface") == m_CustomParameters.end())
        {
            spdlog::error("Filter [{}-{}]: {}: Interface parameter not found in configuration file ",
                getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
            m_bIsEnabled = false;
            return false;
        }
        else
        {
            m_InterfaceType = CRgbdUtils::RGBDCameraApiType(StringToEnumType(m_CustomParameters["interface"]));
        }
        
        if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_SIM)
        {
            // Read the voxels file
            if (!m_CustomParameters["voxels"].empty())
            {
                fs::path voxels_path = fs::path(getGlobalBasePath()) / fs::path(m_CustomParameters.at("voxels"));
                if (!CDepthImageProcessing::readVoxelsFile(voxels_path, m_Voxels))
                    return false;
            }
            m_bIsEnabled = true;
        }
        else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
        {
            m_bIsEnabled = m_RealSense->start();
        }
        else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
        {
#ifndef GENIUS_BOARD
            //m_pOpenNI2API = std::make_unique<COpenNI2API>(static_cast<CPinholeCameraSensorModel*>(this->m_pSensorModel));
            //m_bIsEnabled = m_pOpenNI2API->start();
#else
            spdlog::error("Unsupported platform for OpenNI");
            m_bIsEnabled = false;
#endif // GENIUS_BOARD
        }
        else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_UNITY_API)
        {
            if (m_CustomParameters.find("ip") == m_CustomParameters.end() || m_CustomParameters.find("port") == m_CustomParameters.end())
            {
                spdlog::error("Filter [{}-{}]: {}: ip and/or port parameters not found in configuration file",
                    getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                return false;
            }
            m_bIsEnabled = m_UnityInterface.enable(m_CustomParameters["ip"], m_CustomParameters["port"]);
        }
    }
    else
    {
        m_bIsEnabled = true;
    }
	
    if (m_bIsEnabled)
        spdlog::info("Filter [{}-{}]: {} enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
    else
        spdlog::error("Filter [{}-{}]: {} enable() fail", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());

	return m_bIsEnabled;

}

bool CRgbdCameraFilter::disable()
{	
	if (isRunning())
        stop();

	if (!isNetworkFilter())
    {
        if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
        {
            m_RealSense->stop();
        }
        else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
        {
#ifndef GENIUS_BOARD
            //m_pOpenNI2API->stop();
#endif // GENIUS_BOARD
        }
    }

	m_bIsEnabled = false;
	return true;
}

bool CRgbdCameraFilter::process()
{
    bool bReturn(false);
    CyC_TIME_UNIT ts;
    cv::Mat rgb;
    cv::Mat depth;

    // Update camera pose
    CPose pose;
    if (CFilterUtils::getPose(m_pPoseDataFilter, m_lastReadTsPose, pose))
        this->m_pSensorModel->updatePose(pose * this->m_pSensorModel->extrinsics());

    if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_SIM)
    {
        CImageProcessing::simulateImg(static_cast<const CPinholeCameraSensorModel*>(this->getSensorModel()), m_Voxels, rgb, depth);
        bReturn = true;
    }
    else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_REALSENSE_API)
    {
        m_RealSense->get_images(ts, rgb, depth);
        bReturn = true;
    }
    else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_OPENNI_API)
    {
#ifndef GENIUS_BOARD
        cv::Mat depth_frame;
        //bReturn = m_pOpenNI2API->grab(rgb, depth_frame);

        depth = cv::Mat(depth_frame.size(), CV_32FC1);
        for (CyC_INT y = 0; y < depth.rows; ++y)
            for (CyC_INT x = 0; x < depth.cols; ++x)
                depth.at<float>(y, x) = (float)depth_frame.at<uint16_t>(y, x) / 1000.f;
#endif // GENIUS_BOARD
    }
    else if (m_InterfaceType == CRgbdUtils::RGBD_CAMERA_UNITY_API)
    {
        while (!m_UnityInterface.process())
        {
            if (!isRunning())
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        rgb = m_UnityInterface.get_rgb();
        depth = m_UnityInterface.get_depth();

        /*if (m_bFlipImage)
        {
            if (!rgb.empty())
                cv::flip(rgb, rgb, 1);
            cv::flip(depth, depth, 1);
        }*/

        bReturn = true;
    }

    if (bReturn)
    {
        CycImages output;
        output.emplace_back(rgb, depth, ts);

        updateData(output);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    
    return bReturn;
}

void CRgbdCameraFilter::loadFromDatastream(const std::string& _datastream_entry, const std::string& _db_root_path)
{
    CPinholeCameraSensorModel* sensor_model = static_cast<CPinholeCameraSensorModel*>(this->m_pSensorModel);
    
    // Update sensor pose
    CPose pose;
    if (CFilterUtils::getPose(m_pPoseDataFilter, m_lastReadTsPose, pose))
        sensor_model->updatePose(pose * sensor_model->extrinsics());

    csv::reader::row row;
    row.parse_line(_datastream_entry, ',');

    // timestamp_start, timestamp_stop, sampling_time, timestamp_image, rgb_img_path, depth_img_path
    enum { TS_STOP, SAMPLING_TIME, TS_IMAGE, RGB_PATH, DEPTH_PATH, NUM };
    if (row.size() != NUM)
    {
        spdlog::error("{}: Wrong number of columns. {} provided, but expected {}.", typeid(*this).name(), row.size(), NUM);
        return;
    }

    CyC_TIME_UNIT ts_image = row.get<CyC_TIME_UNIT>(TS_IMAGE);

    // RGB and depth images
    const auto rgb_img_path = _db_root_path + row.get<std::string>(RGB_PATH);
    const auto depth_img_path = _db_root_path + row.get<std::string>(DEPTH_PATH);

    auto rgb_img = cv::imread(rgb_img_path, cv::IMREAD_ANYCOLOR);
    auto depth_img = cv::imread(depth_img_path, cv::IMREAD_ANYCOLOR);

    if (rgb_img.cols != sensor_model->width() || rgb_img.rows != sensor_model->height())
        cv::resize(rgb_img, rgb_img, cv::Size(sensor_model->width(), sensor_model->height()));

    if (depth_img.cols != sensor_model->width() || depth_img.rows != sensor_model->height())
        cv::resize(depth_img, depth_img, cv::Size(sensor_model->width(), sensor_model->height()));

    CycImages output;
    if (fs::exists(depth_img_path) && !depth_img.empty())
    {
        auto depth = CDepthImageProcessing::image2depth(depth_img);
        output.emplace_back(rgb_img, depth, ts_image);
    }
    else
    {
        output.emplace_back(rgb_img, cv::Mat(), ts_image);
    }

    const auto tTimestampStop  = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime   = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;

    updateData(output, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);
}

int CRgbdCameraFilter::StringToEnumType(const std::string& str_type)
{
    if (str_type.compare("sim") == 0)
        return CRgbdUtils::RGBD_CAMERA_SIM;
    else if (str_type.compare("openni") == 0)
        return CRgbdUtils::RGBD_CAMERA_OPENNI_API;
    else if (str_type.compare("unity") == 0)
        return CRgbdUtils::RGBD_CAMERA_UNITY_API;
    else if (str_type.compare("realsense") == 0)
        return CRgbdUtils::RGBD_CAMERA_REALSENSE_API;

    return CRgbdUtils::RGBD_CAMERA_OPENNI_API;
}
