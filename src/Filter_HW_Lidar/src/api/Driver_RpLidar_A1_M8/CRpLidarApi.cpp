// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CRpLidarApi.h"

#ifdef _WIN32
#define COM_PATH "\\\\.\\com3"
#elif __APPLE__
#define COM_PATH "/dev/tty.SLAB_USBtoUART"
#else
#define COM_PATH "/dev/ttyUSB0"
#endif

CRpLidarApi::CRpLidarApi()
{}

CRpLidarApi::~CRpLidarApi()
{}

bool CRpLidarApi::init()
{
	this->m_drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
	if (!m_drv)
	{
		return false;
	}
	else
	{
		u_result op_result;
		rplidar_response_device_health_t healthInfo;

		rplidar_response_device_info_t devInfo;
		size_t baudRateArraySize = sizeof(baudRateArray) / sizeof(baudRateArray[0]);
		for (size_t i = 0; i < baudRateArraySize; ++i)
		{
			if (!m_drv)
				m_drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
			if (IS_OK(m_drv->connect(COM_PATH, baudRateArray[i])))
			{
				op_result = m_drv->getDeviceInfo(devInfo);

				if (IS_OK(op_result))
				{
					//spdlog::info("GETTING DEVICE INFO");
					break;
				}
				else
				{
					delete m_drv;
					m_drv = NULL;
					//spdlog::error("COULD NOT CREATE RPLIDAR DRIVER DURING THE BAUDRATEARRAY ITERATION 4");
					return false;
				}
			}
		}

		op_result = m_drv->getHealth(healthInfo);
		if (IS_OK(op_result))
		{
			if (healthInfo.status == RPLIDAR_STATUS_ERROR)
			{
				//spdlog::error("RPLIDAR_HEALTH_ERROR 2");
				return false;
			}
			else {
				//return true;
			}
		}
		else
		{
			//spdlog::error("COULD NOT RETRIVE RPLIDAR HEALTH INFORMATION 3");
			return false;
		}

		m_bInitialised = true;
	}

	return m_bInitialised;
}

bool CRpLidarApi::start()
{
	if (m_bInitialised)
	{
		m_drv->startMotor();
		m_drv->startScan(0, 1);
		m_bRunning = true;
	}
	else
	{
		m_bRunning = false;
	}

	return m_bRunning;
}

bool CRpLidarApi::stop()
{
	m_drv->stop();
	m_drv->stopMotor();
	m_bRunning = false;
	return true;
}

bool CRpLidarApi::scan(rplidar_response_measurement_node_hq_t* nodes, size_t count)
{
	if (!m_bRunning)
	{
		return false;
	}
	else
	{
		u_result op_result;
		//rplidar_response_measurement_node_hq_t nodes[8192];
		//size_t count = _countof(nodes);

		op_result = m_drv->grabScanDataHq(nodes, count);

		if (IS_OK(op_result))
		{
			m_drv->ascendScanData(nodes, count);

			//m_LidarVoxels.clear();
			//for (int index = 0; index < count; ++index) {
			//	CycLidarPoint CurrentLidarPoint;
			//	CurrentLidarPoint.distance = (nodes[index].dist_mm_q2 / 10.0f) / (1 << 2);
			//	CurrentLidarPoint.angle = (nodes[index].angle_z_q14 * 90.0f) / (1 << 14);
			//	CurrentLidarPoint.quality = nodes[index].flag >> RPLIDAR_RESP_HQ_FLAG_SYNCBIT;
			//	m_LidarVoxels.push_back(CurrentLidarPoint);
			//}
		}

		return true;
	}
}