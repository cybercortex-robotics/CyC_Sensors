#ifndef CyC_UBLOX_GPS_INTERFACE_H_
#define CyC_UBLOX_GPS_INTERFACE_H_

#include <memory>
#include <string>

enum class ERTKSolutionType
{
    None,
    DGNSS_Float,
    DGNSS_Fix
};

enum class EFixType
{
    None,
    Fix_Dead_reckoning,
    Fix_2D,
    Fix_3D,
    Fix_GNSS,
    Fix_Time
};

struct SGpsData
{
    struct {
        uint8_t day = 0;
        uint8_t month = 0;
        uint16_t year = 0;
        uint8_t hour = 0;
        uint8_t minute = 0;
        uint8_t second = 0;
        uint16_t millisecond = 0;
        int32_t nanosecond = 0;
    } date;

    ERTKSolutionType rtk_type = ERTKSolutionType::None;
    EFixType fix_type = EFixType::None;

    double lat = 0.; // latitude
    double lng = 0.; // longitude
    double alt = 0.; // altitude
    double alt_msl = 0.; // altitude at mean sea level

    uint8_t num_sat = 0; // satellites
    double pdop = 0.; // positional dillution of precision * 10^-2 (dimensionless)
    double ground_speed = 0.; // ground speed
    double ned_northvel = 0.;
    double ned_eastvel = 0.;
    double ned_downvel = 0.;
    double vertical_acc_est = 0.;
    double horizontal_acc_est = 0.;
    double speed_acc_est = 0.;
    uint32_t heading_acc_est = 0.;
    bool head_veh_valid = false;
    int32_t head_veh = 0.;
    int16_t mag_dec = 0.;
    uint16_t mag_acc = 0.;
};

class UbloxGPS
{
public:
    UbloxGPS();
    ~UbloxGPS();

    bool begin(size_t port_index);
    bool begin(const std::string& port_name);

    std::unique_ptr<SGpsData> getData();

private:
    struct impl;
    impl* m_impl;
};

#endif // CyC_UBLOX_GPS_INTERFACE_H_
