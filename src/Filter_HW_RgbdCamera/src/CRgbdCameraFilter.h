// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CRgbdCameraFilter_H_
#define CRgbdCameraFilter_H_

#include "CyC_TYPES.h"
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "os/CSingletonRegistry.h"
#include "CCycFilterBase.h"
#include "CRgbdUtils.h"
#include "CRealSense2Api.h"
//#include "COpenNI2API.h"
#include "api/unity/CRgbdCameraUnityApi.h"

class CRgbdCameraFilter : public CCycFilterBase
{
public:
	explicit CRgbdCameraFilter(CycDatablockKey _key);
	explicit CRgbdCameraFilter(const ConfigFilterParameters& _params);

	~CRgbdCameraFilter() override;

	bool enable() override;
	bool disable() override;
	
private:
	bool		process() override;
    void        loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;
	static int  StringToEnumType(const std::string& str_type);

private:
	CRgbdUtils::RGBDCameraApiType	m_InterfaceType = CRgbdUtils::RGBD_CAMERA_OPENNI_API;

	//std::unique_ptr<COpenNI2API>	m_pOpenNI2API;
	CRGBDCameraUnityApi				m_UnityInterface;

	std::shared_ptr<CRealSense2Api> m_RealSense;

	bool m_bRectifyImages;

	// Updates pose of the camera
	CCycFilterBase*	m_pPoseDataFilter = nullptr;
	CyC_TIME_UNIT	m_lastReadTsPose = 0;

	// Voxels (in world coordinates system) used for simulated camera
	CycVoxels		m_Voxels;
};

#endif /* CRgbdCameraFilter_H_ */
