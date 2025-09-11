// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CGpsFilter.h"
#include "rs232.h"
#include <os/CCsvReader.h>
#include "interfaces/ublox_gps/UbloxGPS.h"

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
	CycDatablockKey key;
	return CGpsFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
	return new CGpsFilter(_params);
}

CGpsFilter::CGpsFilter(CycDatablockKey key) :
    CCycFilterBase(key)
{
	// Assign the filter type, input type and output type
	setFilterType("CyC_GPS_FILTER_TYPE");
	m_OutputDataType = CyC_GPS;
}

CGpsFilter::CGpsFilter(const ConfigFilterParameters& params) :
    CCycFilterBase(params)
{
	// Assign the filter type, input type and output type
	setFilterType("CyC_GPS_FILTER_TYPE");
	m_OutputDataType = CyC_GPS;

	if (!m_CustomParameters["interface"].empty())
	{
		m_InterfaceType = StringToEnumType(m_CustomParameters["interface"]);
	}
	else
	{
		spdlog::warn("Filter [{}-{}]: CGpsFilter: No gps interface type defined. Switching to simulation.", getFilterKey().nCoreID, getFilterKey().nFilterID);
		m_InterfaceType = Gps_SIM_INTERFACE;
	}

	if (m_InterfaceType == Gps_REAL_INTERFACE || m_InterfaceType == Gps_UBLOX_INTERFACE)
	{
		if (!m_CustomParameters["PortNr"].empty())
		{
			m_portNr = std::stoi(m_CustomParameters["PortNr"]);
		}
		else
		{
			spdlog::error("Filter [{}-{}]: CGpsFilter: Port number undefined.", getFilterKey().nCoreID, getFilterKey().nFilterID);
		}
	}

	// Simulation params
	m_RandomGen = std::make_unique<std::mt19937>(m_RandomDevice());
}

CGpsFilter::~CGpsFilter()
{
	if (m_bIsEnabled)
		disable();
}

bool CGpsFilter::enable()
{
	if (!isNetworkFilter() && !isReplayFilter())
	{
		if (m_InterfaceType == Gps_REAL_INTERFACE)
		{
			if (!isNetworkFilter())
			{
				const char mode[] = { '8','N','1', 0 };
				const bool gps_connected = !RS232_OpenComport(m_portNr, 9600, mode, 0);

				if (!gps_connected)
				{
					spdlog::error("Filter [{}-{}]: CGps::Can't open port {}", getFilterKey().nCoreID, getFilterKey().nFilterID, m_portNr);
					m_bIsEnabled = false;
					return false;
				}
			}
		}
		else if (m_InterfaceType == Gps_SIM_INTERFACE)
		{
			for (const CycInputSource& src : this->getInputSources())
			{
				if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE"))
					m_pStateSimFilter = src.pCycFilter;
				else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE"))
					m_pStateSimFilter = src.pCycFilter;
				else
				{
					spdlog::error("Filter [{}-{}]: CGpsFilter: Expected CyC_VEHICLE_SIMULATION_FILTER_TYPE or CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE as input filter. CRotaryEncoderFilter disabled.", getFilterKey().nCoreID, getFilterKey().nFilterID);
					return false;
				}
			}

			if (m_pStateSimFilter == nullptr)
			{
				spdlog::error("Filter [{}-{}]: CGpsFilter::enable(): No input state measurement sources for simulation. Disabling CGpsFilter.", getFilterKey().nCoreID, getFilterKey().nFilterID);
				return false;
			}
		}
		else if (m_InterfaceType == Gps_BRICK_INTERFACE)
		{
			/*if (!m_gpsBricklet.setup(1000)) 
			{
				spdlog::error("Filter[{}-{}]: Can't setup brick imu.");
				return false;
			}

			std::this_thread::sleep_for(std::chrono::seconds(1));
			m_gpsBricklet.start();*/
		}
		else if (m_InterfaceType == Gps_UBLOX_INTERFACE)
		{
			m_ublox_gps = std::make_unique<UbloxGPS>();
			if (!m_ublox_gps->begin(m_portNr))
			{
				return false;
			}
		}
	}

	spdlog::info("Filter [{}-{}]: CGps::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID);

	m_bIsEnabled = true;
	return true;
}


bool CGpsFilter::disable()
{
	if (isRunning())
		stop();

	if (!isNetworkFilter() && m_InterfaceType == Gps_REAL_INTERFACE)
		RS232_CloseComport(m_portNr);

	m_bIsEnabled = false;
	return true;
}


bool CGpsFilter::process()
{
	bool bReturn(false);
	CycGps gps;
	
	if (m_InterfaceType == Gps_SIM_INTERFACE)
	{
		// Simulate GPS
		const auto tsState = m_pStateSimFilter->getTimestampStop();
		
		if (tsState > m_lastTsState)
		{
			CycState state;
			if (m_pStateSimFilter->getData(state))
			{
				// instance of class std::normal_distribution with specific mean and stddev
				std::normal_distribution<float> p_lat(0.f, 1.5f);
				std::normal_distribution<float> p_lon(0.f, 1.5f);

				// The offset is added because the UTM coordinates
				// can't be negative.
				const float x_offset = 10000.F;
				const float y_offset = 10000.F;

				state.x_hat[0] += p_lat(*m_RandomGen);
				state.x_hat[1] += p_lon(*m_RandomGen);

				Eigen::Vector2f sim_gps = CGeolocation::utm2gps(state.x_hat[0] + x_offset, state.x_hat[1] + y_offset, 0.F);
				gps.lat = sim_gps.x();
				gps.lng = sim_gps.y();
				gps.alt = 0.f;

				bReturn = true;
			}
		}
	}
	else if (m_InterfaceType == Gps_REAL_INTERFACE)
	{
		std::this_thread::sleep_for(std::chrono::microseconds(10));
		std::string line = readSerialLine();

		if (line.find("$GNGGA") != std::string::npos)
		{
			std::vector<std::string> vec = splitLine(line);

			if (isdigit(vec[2][0]) && isdigit(vec[4][0]) && isdigit(vec[9][0])) // lat, lng and altitude sit at positions 2,4 and 9 in $GNGGA sentence
			{
				gps.lat = CGeolocation::nmea2Deg(stof(vec[2]));
				gps.lng = CGeolocation::nmea2Deg(stof(vec[4]));
				gps.alt = stof(vec[9]);

				bReturn = true;
			}
		}

		//else if (line.find("$GNRMC") != std::string::npos) {
		//	std::vector<std::string> vec = splitLine(line);
		//	if (isdigit(vec[7][0])) {  // speeds sits at positions 7 in $GNRMC sentence
		//		m_GpsData.speed = stof(vec[7]) * 1.852; //knots to kmh
		//	}
		//}
	}
	else if (m_InterfaceType == Gps_BRICK_INTERFACE)
	{
		/*const auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now().time_since_epoch()).count();

		if ((ts - m_lastTsState) > 10)
		{
			m_lastTsState = ts;

			m_gpsBricklet.lock();
			const CGpsBricklet::gps_data gpsData = m_gpsBricklet.getData();
			m_gpsBricklet.unlock();

			gps.lat = gpsData.lat;
			gps.lng = gpsData.lng;
			gps.alt = gpsData.alt;

			bReturn = true;
		}*/
	}
	else if (m_InterfaceType == Gps_UBLOX_INTERFACE)
	{
		auto data = m_ublox_gps->getData();
		if (data)
		{
			gps.lat = data->lat;
			gps.lng = data->lng;
			gps.alt = data->alt;

			bReturn = true;
		}
	}
	
	if (bReturn)
	{
		if (!m_IsGPSInitialized)
		{
			m_IsGPSInitialized = true;
			m_InitialGPS = gps;
		}

		gps.baseLat = m_InitialGPS.lat;
		gps.baseLng = m_InitialGPS.lng;
		gps.baseAlt = m_InitialGPS.alt;

		//spdlog::info("Latitude, longitude: {}, {}", gps.lat, gps.lng);
		//spdlog::info("Altitude: {}", gps.alt);

		updateData(gps);
	}

    return bReturn;
}

void CGpsFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
	// Set the processing flag
	m_bIsProcessing = true;

	csv::reader::row row;
	row.parse_line(datastream_entry, ',');

	//timestamp_start,timestamp_stop,sampling_time,lat,lng,alt
	enum { TS_STOP, SAMPLING_TIME, LAT, LNG, ALT, NUM };
	if (row.size() < NUM)
	{
		spdlog::error("Filter [{}-{}]: Wrong number of columns. {} provided, but expected {}", getFilterKey().nCoreID, getFilterKey().nFilterID, row.size(), NUM + 1);
		return;
	}

	CycGps gps;

	const auto tTimestampStop  = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime   = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;
    
	gps.lat = row.get<float>(LAT);
	gps.lng = row.get<float>(LNG);
	gps.alt = row.get<float>(ALT);

	if (!m_IsGPSInitialized)
	{
		m_IsGPSInitialized = true;
		m_InitialGPS = gps;
	}

	gps.baseLat = m_InitialGPS.lat;
	gps.baseLng = m_InitialGPS.lng;
	gps.baseAlt = m_InitialGPS.alt;

	updateData(gps, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);

	// Unset the processing flag
	m_bIsProcessing = false;
}

std::string CGpsFilter::readSerialLine()
{
	unsigned char ch = 0;
	std::string line;

	while (ch != 10)
		if (RS232_PollComport(m_portNr, &ch, 1))
			line += ch;

	return line;
}

std::vector<std::string> CGpsFilter::splitLine(const std::string& line)
{
	std::stringstream sstream(line);
	std::vector<std::string> vec;
	std::string temp;
	while (getline(sstream, temp, ','))
		vec.push_back(temp);
	return vec;
}

int CGpsFilter::StringToEnumType(const std::string& str_ctrl_name)
{
	if (str_ctrl_name.compare("sim") == 0)
		return Gps_SIM_INTERFACE;
	else if (str_ctrl_name.compare("real") == 0)
		return Gps_REAL_INTERFACE;
	else if (str_ctrl_name.compare("brick") == 0)
		return Gps_BRICK_INTERFACE;
	else if (str_ctrl_name == "ublox")
		return Gps_UBLOX_INTERFACE;

	return Gps_SIM_INTERFACE;
}
