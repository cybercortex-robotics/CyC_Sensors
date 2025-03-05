// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "COpenNI2API.h"

COpenNI2API::COpenNI2API(CPinholeCameraSensorModel* _sensor_model, bool _flip_images, bool _sync_rgb_depth) :
	m_bFlipImages(_flip_images),
	m_bSyncRgbDepth(_sync_rgb_depth)
{
    m_pSensorModel = _sensor_model;
}

COpenNI2API::~COpenNI2API()
{
	
}

bool COpenNI2API::start()
{
	openni::Status rc = openni::STATUS_OK;

	const char* deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();

	spdlog::info("COpenNI2API::start(): After initialization: {}", openni::OpenNI::getExtendedError());

	rc = m_device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		spdlog::info("COpenNI2API::start(): Device open failed: {}", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return false;
	}

	rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = m_depthStream.start();
		if (rc != openni::STATUS_OK)
		{
			spdlog::info("COpenNI2API::start(): Couldn't start depth stream: {}", openni::OpenNI::getExtendedError());
			m_depthStream.destroy();
		}
	}
	else
	{
		spdlog::info("COpenNI2API::start(): Couldn't find depth stream: {}", openni::OpenNI::getExtendedError());
	}

	rc = m_colorStream.create(m_device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = m_colorStream.start();
		if (rc != openni::STATUS_OK)
		{
			spdlog::info("COpenNI2API::start(): Couldn't start color stream: {}", openni::OpenNI::getExtendedError());
			m_colorStream.destroy();
		}
	}
	else
	{
		spdlog::info("COpenNI2API::start(): Couldn't find color stream: {}", openni::OpenNI::getExtendedError());
	}

	if (!m_depthStream.isValid() || !m_colorStream.isValid())
	{
		spdlog::info("COpenNI2API::start(): No valid streams. Exiting");
		openni::OpenNI::shutdown();
		return false;
	}
	
	// Setup the depth stream
	openni::VideoMode vmDepth;
	vmDepth.setResolution(m_pSensorModel->width(), m_pSensorModel->height());
	vmDepth.setFps(30);
	vmDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	if (m_depthStream.setVideoMode(vmDepth) != 0)
	{
		spdlog::error("COpenNI2API::start(): ERROR - Could not configure depth sensor video stream");
	}

	// Setup the color stream
	openni::VideoMode vmColor;
	vmColor.setResolution(m_pSensorModel->width(), m_pSensorModel->height());
	vmColor.setFps(30);
	vmColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	if (m_colorStream.setVideoMode(vmColor) != 0)
	{
		spdlog::error("COpenNI2API::start(): ERROR - Could not configure color sensor video stream");
	}


	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;

	if (m_depthStream.isValid() && m_colorStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		colorVideoMode = m_colorStream.getVideoMode();

		int depthWidth = depthVideoMode.getResolutionX();
		int depthHeight = depthVideoMode.getResolutionY();
		int colorWidth = colorVideoMode.getResolutionX();
		int colorHeight = colorVideoMode.getResolutionY();

		//std::cout << depthWidth << "    " << depthHeight << "     " << colorWidth << "    " << colorHeight << std::endl;

		if (depthWidth == colorWidth && depthHeight == colorHeight)
		{
			//m_width = depthWidth;
			//m_height = depthHeight;
		}
		else
		{
			spdlog::info("COpenNI2API::start(): Error - expect color and depth to be in same resolution: D: {}x{}, C: {}x{}",
				depthWidth, depthHeight,
				colorWidth, colorHeight);
			return openni::STATUS_ERROR;
		}
	}
	else if (m_depthStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		//m_width = depthVideoMode.getResolutionX();
		//m_height = depthVideoMode.getResolutionY();
	}
	else if (m_colorStream.isValid())
	{
		colorVideoMode = m_colorStream.getVideoMode();
		//m_width = colorVideoMode.getResolutionX();
		//m_height = colorVideoMode.getResolutionY();
	}
	else
	{
		spdlog::info("COpenNI2API::start(): Error - expects at least one of the streams to be valid.");
		return openni::STATUS_ERROR;
	}

	//std::cout << m_depthFrame.getWidth() << "    " << m_depthFrame.getHeight() << "     " << m_colorFrame.getWidth() << "    " << m_colorFrame.getHeight() << std::endl;

	// Enable depth/color frame synchronization
	if (m_bSyncRgbDepth)
	{
		rc = m_device.setDepthColorSyncEnabled(true);
		if (rc != openni::STATUS_OK)
		{
			spdlog::info("COpenNI2API::start(): Could not synchronise device: {}", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return false;
		}
		else
		{
			spdlog::info("COpenNI2API::start(): RGB and Depth images are synchronized", openni::OpenNI::getExtendedError());
		}
	}

	m_streams = new openni::VideoStream * [2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;

	return true;
}

bool COpenNI2API::stop()
{
	m_depthStream.stop();
	m_colorStream.stop();
	m_depthStream.destroy();
	m_colorStream.destroy();
	m_device.close();
	openni::OpenNI::shutdown();

	return true;
}

bool COpenNI2API::grab(cv::Mat& _rgb, cv::Mat& _depth)
{
	CyC_INT dummy_idx;

	openni::Status rc = openni::OpenNI::waitForAnyStream(&m_streams[1], 1, &dummy_idx);
	if (rc != openni::STATUS_OK)
	{
		spdlog::info("COpenNI2API::grab(): Wait failed");
		return false;
	}
	
	rc = m_colorStream.readFrame(&m_colorFrame);
	if (rc != openni::STATUS_OK)
	{
		spdlog::info("COpenNI2API::grab(): Read RGB failed!\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}

	rc = openni::OpenNI::waitForAnyStream(&m_streams[0], 1, &dummy_idx);
	if (rc != openni::STATUS_OK)
	{
		spdlog::info("COpenNI2API::grab(): Wait failed");
		return false;
	}

	rc = m_depthStream.readFrame(&m_depthFrame);
	if (rc != openni::STATUS_OK)
	{
		spdlog::info("COpenNI2API::grab(): Read depth failed!\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}

	// grab the RGB image
	if (m_colorFrame.isValid())
	{
		const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)m_colorFrame.getData();
		_rgb.create(m_colorFrame.getHeight(), m_colorFrame.getWidth(), CV_8UC3);
		memcpy(_rgb.data, imageBuffer, 3 * (CyC_INT)m_colorFrame.getHeight() * (CyC_INT)m_colorFrame.getWidth() * sizeof(uint8_t));
		cv::cvtColor(_rgb, _rgb, cv::COLOR_BGR2RGB);
	}

	// grab the depth image
	if (m_depthFrame.isValid())
	{
		const openni::DepthPixel* imageBuffer2 = (const openni::DepthPixel*)m_depthFrame.getData();
		_depth.create(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
		memcpy(_depth.data, imageBuffer2, (CyC_INT)m_depthFrame.getHeight() * (CyC_INT)m_depthFrame.getWidth() * sizeof(uint16_t));
	}

	if (m_bFlipImages)
	{
		cv::flip(_rgb, _rgb, 1);
		cv::flip(_depth, _depth, 1);
	}

	return m_colorFrame.isValid() && m_depthFrame.isValid();
}

/*
bool COpenNI2API::grab(cv::Mat& _rgb, cv::Mat& _depth)
{
	CyC_INT changedIndex;
	openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
	if (rc != openni::STATUS_OK)
	{
		spdlog::info("COpenNI2API::grab(): Wait failed");
		return false;
	}
	std::cout << changedIndex << std::endl;
	switch (changedIndex)
	{
		case 0:
			m_depthStream.readFrame(&m_depthFrame); break;
		case 1:
			m_colorStream.readFrame(&m_colorFrame); break;
		default:
			spdlog::info("COpenNI2API::grab(): Error in wait");
	}

	// grab the RGB image
	if (m_colorFrame.isValid())
	{
		const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)m_colorFrame.getData();
		_rgb.create(m_colorFrame.getHeight(), m_colorFrame.getWidth(), CV_8UC3);
		memcpy(_rgb.data, imageBuffer, 3 * m_colorFrame.getHeight() * m_colorFrame.getWidth() * sizeof(uint8_t));
		cv::cvtColor(_rgb, _rgb, cv::COLOR_BGR2RGB);
	}

	// grab the depth image
	if (m_depthFrame.isValid())
	{
		const openni::DepthPixel* imageBuffer2 = (const openni::DepthPixel*)m_depthFrame.getData();
		_depth.create(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
		memcpy(_depth.data, imageBuffer2, m_depthFrame.getHeight() * m_depthFrame.getWidth() * sizeof(uint16_t));
	}

	if (m_bFlipImages)
	{
		cv::flip(_rgb, _rgb, 1);
		cv::flip(_depth, _depth, 1);
	}

	return m_colorFrame.isValid() && m_colorFrame.isValid();
}
*/
