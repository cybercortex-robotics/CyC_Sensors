// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "gps.h"

CGpsBricklet::~CGpsBricklet()
{
    if (isConnected)
    {
        stop();
    }
}

bool CGpsBricklet::setup(int period)
{
    m_period = period;
    spdlog::info("Connecting to GPS Bricklet... Period: {}", m_period);
    ipcon_create(&m_connection);

    gps_v2_create(&m_gps,GPS_UID, &m_connection);

    isConnected = false;

    if (ipcon_connect(&m_connection, GPS_HOST, GPS_PORT) < 0)
    {
        spdlog::error("Error connecting to GPS Bricklet!");
        return false;
    }
    else
    {
		spdlog::info("Connection to GPS Bricklet: Success!");
    }

    gps_v2_register_callback(&m_gps, GPS_V2_CALLBACK_COORDINATES, (void (*)(void))cb_coordinates, (void*)this);
    gps_v2_register_callback(&m_gps, GPS_V2_CALLBACK_ALTITUDE, (void (*)(void))cb_altitude, (void*)this);
    spdlog::info("Callback registration: Success!");

    return true;
}

// Static callback function
void CGpsBricklet::cb_coordinates(uint32_t latitude, char ns, uint32_t longitude, char ew, void *user_data)
{
    //(void)user_data; // avoid unused parameter warning
    //spdlog::info("Latitude, longitude: {}, {} °", latitude/1000000.F, longitude / 1000000.F);
    //spdlog::info("N/S: {}", ns);
    //spdlog::info("E/W: {}", ew);

    CGpsBricklet& gps = *reinterpret_cast<CGpsBricklet*>(user_data);
    std::lock_guard<std::mutex> guard(gps.m_mutex);

    if (gps.isConnected)
        return;

    gps.m_gpsData.lat = latitude / 1000000.F;
    gps.m_gpsData.lng = longitude / 1000000.F;
}

void CGpsBricklet::cb_altitude(int32_t altitude, int32_t geoidal_separation, void* user_data)
{
    //(void)user_data; // avoid unused parameter warning
    //spdlog::info("Altitude: {} m", altitude / 100.F);

    CGpsBricklet& gps = *reinterpret_cast<CGpsBricklet*>(user_data);
    std::lock_guard<std::mutex> guard(gps.m_mutex);

    gps.m_gpsData.alt = altitude / 100.F;
}

void CGpsBricklet::start()
{
    gps_v2_set_coordinates_callback_period(&m_gps, m_period);
    gps_v2_set_altitude_callback_period(&m_gps, m_period);
}

void CGpsBricklet::suspend()
{
    gps_v2_set_coordinates_callback_period(&m_gps, 0);
    gps_v2_set_altitude_callback_period(&m_gps, 0);
}

void CGpsBricklet::stop()
{
    gps_v2_destroy(&m_gps);
    ipcon_destroy(&m_connection);
    isConnected = false;
}
