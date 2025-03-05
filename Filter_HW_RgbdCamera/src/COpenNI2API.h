// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// https://elcharolin.wordpress.com/2017/09/06/transforming-a-depth-map-into-a-3d-point-cloud/

#ifndef COpenNI2API_H_
#define COpenNI2API_H_

#include "CyC_TYPES.h"
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <OpenNI.h>
#include "sensors/CPinholeCameraSensorModel.h"

class COpenNI2API
{
public:
	explicit COpenNI2API(CPinholeCameraSensorModel* _sensor_model, bool _flip_images = true, bool _sync_rgb_depth = true);
	~COpenNI2API();

	bool start();
	bool stop();
	bool grab(cv::Mat& _rgb, cv::Mat& _depth);

private:
	CPinholeCameraSensorModel*	m_pSensorModel;
	bool					m_bFlipImages;
	bool					m_bSyncRgbDepth;

	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	openni::Device				m_device;
	openni::VideoStream			m_depthStream;
	openni::VideoStream			m_colorStream;
	openni::VideoStream**		m_streams;
};

#endif /* COpenNI2API_H_ */
