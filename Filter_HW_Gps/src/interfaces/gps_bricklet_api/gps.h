// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef GPS_CORE_H
#define GPS_CORE_H

#define IPCON_EXPOSE_INTERNALS
#include "ip_connection.h"
#include "bricklet_gps_v2.h"

#include <spdlog/spdlog-inl.h>

#define GPS_HOST "localhost"
#define GPS_PORT 4223
#define GPS_UID "Pui"

class CGpsBricklet
{
public:

    struct gps_data
    {
		float baseLat;
		float baseLng;
		float baseAlt;
		float lat;
		float lng;
		float alt;
    };
	
    CGpsBricklet() = default;
    ~CGpsBricklet();
    CGpsBricklet(const CGpsBricklet&) = delete;
    CGpsBricklet(CGpsBricklet&&) = delete;
    CGpsBricklet& operator=(const CGpsBricklet&) = delete;
    CGpsBricklet& operator=(CGpsBricklet&&) = delete;

    void lock() { m_mutex.lock(); }
    const gps_data& getData() { return m_gpsData; }
    void unlock() { m_mutex.unlock(); }

    bool setup(int period);
    void start();
    void suspend();
    void stop();

private:
    static void cb_coordinates(uint32_t latitude, char ns, uint32_t longitude, char ew, void *user_data);
    static void cb_altitude(int32_t altitude, int32_t geoidal_separation, void* user_data);

    IPConnection m_connection;
    bool isConnected;
    GPSV2 m_gps;
    int m_period;

    gps_data m_gpsData;

    std::mutex m_mutex;
};

#endif // GPS_CORE_H
