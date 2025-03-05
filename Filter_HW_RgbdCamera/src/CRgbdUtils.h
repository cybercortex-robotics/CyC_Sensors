// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CRgbdUtils_H_
#define CRgbdUtils_H_

#include "CyC_TYPES.h"

class CRgbdUtils
{
public:
	enum RGBDCameraApiType
	{
		RGBD_CAMERA_UNKNOWN = 0,		// Unknown camera type
		RGBD_CAMERA_SIM = 1,			// "sim"
		RGBD_CAMERA_OPENNI_API = 2,		// "openni" - default
		RGBD_CAMERA_UNITY_API = 3,		// "unity"
		RGBD_CAMERA_REALSENSE_API = 4	// "realsense"
	};
    
public:
    explicit CRgbdUtils();
	~CRgbdUtils();
};

#endif /* CRgbdUtils_H_ */
