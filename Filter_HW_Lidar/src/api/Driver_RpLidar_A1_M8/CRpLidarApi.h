// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "rplidar.h"

class CRpLidarApi
{
public:
	CRpLidarApi();
	virtual ~CRpLidarApi();

	bool init();
	bool start();
	bool stop();
	bool scan(rplidar_response_measurement_node_hq_t* nodes, size_t count);

	bool isInitialized() { return m_bInitialised; };
	bool isRunning() { return m_bRunning; };

private:
	bool m_bInitialised = false;
	bool m_bRunning = false;

	rp::standalone::rplidar::RPlidarDriver* m_drv;
	int scans = 0;
	std::vector<rplidar_response_measurement_node_hq_t>	m_AcquiredVoxels;
	const unsigned int baudRateArray[2] = { 115200, 256000 };
};