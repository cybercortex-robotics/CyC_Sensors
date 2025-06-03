// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CLIDARFILTER_H_
#define CLIDARFILTER_H_

#include "CyC_TYPES.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <csv_reader.h>
#include "CCycFilterBase.h"
#include <os/CFilterUtils.h>
#include "sensors/CLidarSensorModel.h"
#include "happly.h"
#include "api/Driver_RpLidar_A1_M8/CRpLidarApi.h"
#include <lslidar.hpp>

class CHesaiLidarInterface;
class CLidarSlamwareInterface;

enum LidarInterfaceType
{
	Lidar_SIM_INTERFACE = 0,
	Lidar_Rp_INTERFACE = 1,
	Lidar_Hessai_INTERFACE = 2,
    Lidar_Slamware_INTERFACE = 3,
	Lidar_LSLidar_INTERFACE = 4,
};

class CLidarFilter : public CCycFilterBase
{
public:
	explicit CLidarFilter(CycDatablockKey key);
	explicit CLidarFilter(const ConfigFilterParameters params);
	~CLidarFilter() override;

	bool enable() override;
	bool disable() override;

private:
	bool process() override;
	void loadFromDatastream(const std::string& _datastream_entry, const std::string& _db_root_path) override;

	bool getLidarPose(CPose& _out_pose);

private:
	CyC_INT	m_InterfaceType = Lidar_SIM_INTERFACE;
	CycVoxels	m_LidarVoxels;

    CHesaiLidarInterface* m_HesaiLidarInterface = nullptr;
    CLidarSlamwareInterface* m_SlamwareLidarInterface = nullptr;

	// Pose input data filter
	CCycFilterBase*	m_pPoseDataFilter = nullptr;
	CyC_TIME_UNIT   m_lastReadTsPose = 0;
	CLsLidar        m_lslidar;

	std::mutex m_mutex;
	std::thread m_thread;
	std::vector<ScanPoint> m_lidar_points;
};

#endif //CLIDARFILTER_H_