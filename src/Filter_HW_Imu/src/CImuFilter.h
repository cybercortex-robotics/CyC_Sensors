// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
* Install device driver CH340G from https://sparks.gogo.co.nz/ch340.html
* Arduino board info: ESP8266 (install doc: https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/)
* Board: LOLIN (WEMOS) D1 R2 & mini
* COM port setup: check device manager and set com port to "com_port - 1", where com_port is the comport in the device manager
*
* Angles convention: 
*   https://en.wikipedia.org/wiki/Aircraft_principal_axes
*   https://en.wikipedia.org/wiki/Euler_angles
*
* Conventions:
*   Z - Yaw
* 
* References:
*   https://stanford.edu/class/ee267/lectures/lecture9.pdf
*   https://www.unibw.de/lrt9/lrt-9.2/preprints/ion2021_c2_simss_lidar_sanchezbochkatischuetz.pdf
*/

#ifndef CImuFilter_H_
#define CImuFilter_H_

#include "CyC_TYPES.h"
#include <fstream>
#include <algorithm>
#include <csv_reader.h>
#include "CCycFilterBase.h"
#include "CImuUtils.h"
#include "os/CFilterUtils.h"
#include "sensors/CImuSensorModel.h"
#include "control/CStateSpaceModelVehicle.h"
#include "api/MPU9250/CImuMPU9250_API.h"
#include "api/brick/CImuBrick_API.h"
#include "api/unity/CImuUnity_API.h"
#include "CImuSim.h"
#include "math/CGeometry.h"
#include "CRealSense2Api.h"

#ifdef __ANDROID_API__
#include <CNativeSensor_API.h>
#endif

class CImuFilter : public CCycFilterBase
{
public:
	explicit CImuFilter(CycDatablockKey _key);
	explicit CImuFilter(const ConfigFilterParameters& _params);
	~CImuFilter() override;

	bool    enable() override;
	bool    disable() override;

private:
	bool    process() override;
	void    loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

    bool    getPose(CPose& _out_pose);
    
    CImuUtils::ImuApiType  StringToEnumType(const std::string& str_type);

private:
    CImuUtils::ImuApiType   m_ApiType = CImuUtils::Imu_API_SIM;
    CycImus                 m_ImuCache;
    
    // Imu simulation
    CImuSim         m_ImuSim;
    CCycFilterBase* m_pVehStateFilter = nullptr;
    CycState        m_VehicleState;
    CyC_TIME_UNIT   m_lastTsState = 0;
    CycState        m_PrevState;    // Used for simulation
    Eigen::Vector2f m_PrevLinVel_W = Eigen::Vector2f(0.f, 0.f);
    Eigen::Vector2f m_PrevLocation = Eigen::Vector2f(0.f, 0.f);
    Eigen::Vector2f m_PrevPrevLocation = Eigen::Vector2f(0.f, 0.f);
    float           m_PrevDt = 0.f;
    
    // APIs
    CImuMPU9250_API m_ImuMPU9250_API;
    std::shared_ptr<CRealSense2Api> m_RealSense;

    // Updates pose of the IMU
    // TODO: check if we need this!!!
    CCycFilterBase* m_pPoseDataFilter = nullptr;
    CyC_TIME_UNIT   m_lastReadTsPose = 0;

    // APIs
    CImuBrick_API   m_ImuBrick_API;
    CImuUnity_API   m_ImuUnity_API;

#ifdef __ANDROID_API__
    NDKSensor* m_ndkSensor;
#endif
};

#endif
