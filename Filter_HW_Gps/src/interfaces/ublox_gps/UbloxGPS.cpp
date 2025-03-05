#include "UbloxGPS.h"
#ifdef WIN32
#include <Windows.h>
#endif // WIN32
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Stream.h>
#include <spdlog/spdlog.h>

static const size_t UBLOX_SERIAL_PORTNR = 32;

#ifdef WIN32

static const char* const UBLOX_COMPORTS[UBLOX_SERIAL_PORTNR] = {
    "\\\\.\\COM1",  "\\\\.\\COM2",  "\\\\.\\COM3",  "\\\\.\\COM4",
    "\\\\.\\COM5",  "\\\\.\\COM6",  "\\\\.\\COM7",  "\\\\.\\COM8",
    "\\\\.\\COM9",  "\\\\.\\COM10", "\\\\.\\COM11", "\\\\.\\COM12",
    "\\\\.\\COM13", "\\\\.\\COM14", "\\\\.\\COM15", "\\\\.\\COM16",
    "\\\\.\\COM17", "\\\\.\\COM18", "\\\\.\\COM19", "\\\\.\\COM20",
    "\\\\.\\COM21", "\\\\.\\COM22", "\\\\.\\COM23", "\\\\.\\COM24",
    "\\\\.\\COM25", "\\\\.\\COM26", "\\\\.\\COM27", "\\\\.\\COM28",
    "\\\\.\\COM29", "\\\\.\\COM30", "\\\\.\\COM31", "\\\\.\\COM32"
};

#else // WIN32

static const char* const UBLOX_COMPORTS[UBLOX_SERIAL_PORTNR] = {
    "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS5",
    "/dev/ttyS6", "/dev/ttyS7", "/dev/ttyS8", "/dev/ttyS9", "/dev/ttyS10", "/dev/ttyS11",
    "/dev/ttyS12", "/dev/ttyS13", "/dev/ttyS14", "/dev/ttyS15", "/dev/ttyUSB0",
    "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4", "/dev/ttyUSB5",
    "/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyACM0", "/dev/ttyACM1",
    "/dev/rfcomm0", "/dev/rfcomm1", "/dev/ircomm0", "/dev/ircomm1",
    "/dev/cuau0", "/dev/cuau1"/*, "/dev/cuau2", "/dev/cuau3",
    "/dev/cuaU0", "/dev/cuaU1", "/dev/cuaU2", "/dev/cuaU3"*/
};

#endif // WIN32

struct UbloxGPS::impl
{
    SFE_UBLOX_GNSS m_gps;
    std::unique_ptr<Stream> m_serial;
};

UbloxGPS::UbloxGPS()
    : m_impl{ new UbloxGPS::impl }
{
}

UbloxGPS::~UbloxGPS()
{
    delete m_impl;
}

bool UbloxGPS::begin(size_t port_index)
{
    if (port_index >= UBLOX_SERIAL_PORTNR)
    {
        spdlog::error("UbloxGPS: Port index is out of range.");
        return false;
    }

    return begin(UBLOX_COMPORTS[port_index]);
}

bool UbloxGPS::begin(const std::string& port_name)
{
    m_impl->m_serial = std::make_unique<Stream>(port_name.c_str());
    m_impl->m_serial->begin(38400);

    if (!m_impl->m_serial->isConnected())
    {
        spdlog::error("UbloxGPS: Failed to connect to serial port on {}", port_name);
        return false;
    }

    if (!m_impl->m_gps.begin(*(m_impl->m_serial)))
    {
        spdlog::error("UbloxGPS: Failed to start GPS");
        return false;
    }

    m_impl->m_gps.setNavigationFrequency(8); // Set output to 8 times a second
    m_impl->m_gps.saveConfiguration(); // Save the current settings to flash and BBR

    return true;
}

std::unique_ptr<SGpsData> UbloxGPS::getData()
{
    if (!m_impl->m_gps.getPVT())
    {
        spdlog::error("UbloxGPS: getPVT failed!");
        return nullptr;
    }

    auto data = std::make_unique<SGpsData>();

    data->date.day = m_impl->m_gps.getDay();
    data->date.month = m_impl->m_gps.getMonth();
    data->date.year = m_impl->m_gps.getYear();
    data->date.hour = m_impl->m_gps.getHour();
    data->date.minute = m_impl->m_gps.getMinute();
    data->date.second = m_impl->m_gps.getSecond();
    data->date.millisecond = m_impl->m_gps.getMillisecond();
    data->date.nanosecond = m_impl->m_gps.getNanosecond();

    data->lat = m_impl->m_gps.getLatitude() * 1e-7;
    data->lng = m_impl->m_gps.getLongitude() * 1e-7;
    data->alt = m_impl->m_gps.getAltitude() * 1e-3; // mm to m
    data->alt_msl = m_impl->m_gps.getAltitudeMSL() * 1e-3; // mm to m

    data->num_sat = m_impl->m_gps.getSIV();
    data->pdop = m_impl->m_gps.getPDOP() * 1e-2; // ?
    data->fix_type = static_cast<EFixType>(m_impl->m_gps.getFixType());
    data->rtk_type = static_cast<ERTKSolutionType>(m_impl->m_gps.getCarrierSolutionType());
    data->ground_speed = m_impl->m_gps.getGroundSpeed() * 1e-3; // mm/s to m/s
    data->ned_northvel = m_impl->m_gps.getNedNorthVel() * 1e-3; // mm/s to m/s
    data->ned_eastvel = m_impl->m_gps.getNedEastVel() * 1e-3; // mm/s to m/s
    data->ned_downvel = m_impl->m_gps.getNedDownVel() * 1e-3; // mm/s to m/s
    data->vertical_acc_est = m_impl->m_gps.getVerticalAccEst() * 1e-3; // mm to m
    data->horizontal_acc_est = m_impl->m_gps.getHorizontalAccEst() * 1e-3; // mm to m
    data->speed_acc_est = m_impl->m_gps.getSpeedAccEst() * 1e-3; // mm/s to m/s
    data->heading_acc_est = m_impl->m_gps.getHeadingAccEst();
    data->head_veh_valid = m_impl->m_gps.getHeadVehValid();
    data->head_veh = m_impl->m_gps.getHeadVeh();
    data->mag_acc = m_impl->m_gps.getMagAcc();
    data->mag_dec = m_impl->m_gps.getMagDec();

    return std::move(data);
}
