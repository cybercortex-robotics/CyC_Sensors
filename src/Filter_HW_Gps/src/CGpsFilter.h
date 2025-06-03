// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// Driver: CP210x_Universal_Windows_Driver

#ifndef CGPS_H_
#define CGPS_H_

#include <fstream>
#include <random>
#include "CCycFilterBase.h"
#include "env/CGeolocation.h"
//#include "interfaces/gps_bricklet_api/gps.h"

class UbloxGPS;

class CGpsFilter : public CCycFilterBase
{
public:
	enum GpsInterfaceType
	{
		Gps_SIM_INTERFACE = 0,
		Gps_REAL_INTERFACE = 1,
		Gps_BRICK_INTERFACE = 2,
		Gps_UBLOX_INTERFACE = 3,
	};
	static int StringToEnumType(const std::string& str_ctrl_name);

public:
	explicit CGpsFilter(CycDatablockKey key);
	explicit CGpsFilter(const ConfigFilterParameters& params);
	~CGpsFilter() override;

	bool enable() override;
	bool disable() override;

private:
	bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

	std::string						readSerialLine();
	static std::vector<std::string>	splitLine(const std::string& line);

private:
	CyC_INT m_InterfaceType = Gps_SIM_INTERFACE;
	CyC_INT m_portNr;

	CycGps m_InitialGPS;
	bool m_IsGPSInitialized = false;

	// GPS simulation variables
	CCycFilterBase*					m_pStateSimFilter = nullptr;
	CyC_TIME_UNIT					m_lastTsState = 0;
	std::random_device				m_RandomDevice;		// Random device class instance, source of 'true' randomness for initializing random seed
	std::unique_ptr<std::mt19937>	m_RandomGen;		// Mersenne twister PRNG, initialized with seed from previous random device instance

	//CGpsBricklet				m_gpsBricklet;
	std::unique_ptr<UbloxGPS>	m_ublox_gps;
};
#endif