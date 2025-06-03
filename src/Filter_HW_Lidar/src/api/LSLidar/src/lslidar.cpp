#include <lslidar.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cstddef>
#include <iostream>
#include "serial.hpp"

#undef min
#undef max

struct CLsLidar::impl
{
    impl() = default;
    ~impl();

    using clock_type = std::chrono::steady_clock;

    const std::int32_t data_bits_start = 8;
    const std::int32_t degree_bits_start = 4;
    const std::int32_t rpm_bits_start = 6;
	const std::int32_t baud_rate_ = 500000;
    const std::int32_t points_size_ = 2000;

    bool connected = false;
    bool is_start = false;
    bool high_reflection = false;
    bool compensation = false;
    bool first_compensation = true;

    double min_range = 0.3;
    double max_range = 100.;
    double angle_disable_min = 0.;
    double angle_disable_max = 0.;
    double angle_able_min = 0.;
    double angle_able_max = 360.;
    double last_degree = 0.;	
    double degree_compensation = 0.;

    const uint16_t PACKET_SIZE = 160;
    const std::string lidar_name = "M10_P";

    std::vector<ScanPoint> scan_points_;

    bool open_serial(const std::string& port);
    bool init();
    bool polling();
    void difop_processing(const std::vector<std::byte>& packet_bytes);
    void data_processing(const std::vector<std::byte>& packet_bytes);
    std::size_t receive_data(std::vector<std::byte>& packet_bytes) const;

    serial::Serial serial_;
};

bool CLsLidar::impl::open_serial(const std::string& port)
{
    connected = serial_.open(port, baud_rate_, serial::DataBits::bits_8, serial::Parity::none, serial::StopBits::bits_1);
    return connected;
}

bool CLsLidar::impl::init()
{
    if (!connected)
        return false;

    is_start = !is_start; // toggle

    std::vector<std::byte> packet{ 188, std::byte{0x00} };
    packet[0] = std::byte{ 0xA5 };
    packet[1] = std::byte{ 0x5A };
    packet[2] = std::byte{ 0x55 };
    packet[184] = std::byte{ 0x01 };
    packet[185] = static_cast<std::byte>(is_start);
    packet[186] = std::byte{ 0xFA };
    packet[187] = std::byte{ 0xFB };

    is_start = serial_.write_exact(packet);
    if (is_start)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return is_start;
}

CLsLidar::impl::~impl()
{
    (void)init(); // same packet used to stop the lidar
}

namespace {

    std::int32_t reinterpret(std::byte s, std::byte z)
    {
        const auto s_ = static_cast<std::int32_t>(s);
        const auto z_ = static_cast<std::int32_t>(z);

        return (s_ * 256) + z_;
    }

    template <typename T>
    T clamp(T min, T value, T max)
    {
        return std::min(std::max(value, min), max);
    }

}

std::size_t CLsLidar::impl::receive_data(std::vector<std::byte>& packet_bytes) const
{
    if (!serial_.append_read_exact_into(packet_bytes, 1)) return 0;
    if (packet_bytes[0] != std::byte{0xA5})
    {
        //std::cerr << "Invalid packet byte 0: " << std::hex << (std::uint16_t)packet_bytes[0] << std::endl;
        return 0;
    }

    if (!serial_.append_read_exact_into(packet_bytes, 1)) return 0;
    if (packet_bytes[1] != std::byte{0x5A})
    {
        //std::cerr << "Invalid packet byte 1: " << std::hex << (std::uint16_t)packet_bytes[1] << std::endl;
        return 0;
    }

    if (!serial_.append_read_exact_into(packet_bytes, 2)) return 0;
    std::int32_t len = reinterpret(packet_bytes[2], packet_bytes[3]);

    if ((packet_bytes[2] == std::byte{0x55}) && (packet_bytes[3] == std::byte{0x00}))
    {
        len = 188;
    }

    if (!serial_.append_read_exact_into(packet_bytes, len - packet_bytes.size())) return 0;
    return packet_bytes.size();
}

void CLsLidar::impl::difop_processing(const std::vector<std::byte>& packet_bytes)
{
    const auto s = static_cast<std::uint32_t>(packet_bytes[173]);
    const auto z = static_cast<std::uint32_t>(packet_bytes[174]);

    const std::uint32_t degree_temp = s & 0x7F;
    const std::uint32_t sign_temp = s & 0x80;

    degree_compensation = (sign_temp ? -1. : 1.) * (degree_temp * 256 + z) / 100.;
    first_compensation = false;
}

void CLsLidar::impl::data_processing(const std::vector<std::byte>& packet_bytes)
{
    const double degree_interval = 15.;
    const auto deg = reinterpret(
        packet_bytes[degree_bits_start],
        packet_bytes[degree_bits_start + 1]);

    double degree = deg / 100. + degree_compensation;

    while (degree < 0)
    {
        degree += 360.;
    }

    while (degree > 360.)
    {
        degree -= 360.;
    }

    const std::int32_t package_points = (packet_bytes.size() - 20) / 2;
    const std::int32_t point_len = 2;
    std::int32_t invalid_value = 0;
    for (std::int32_t i = 0; i < (package_points * point_len); i += point_len)
    {
        const auto num = reinterpret(
            packet_bytes[i + data_bits_start],
            packet_bytes[i + data_bits_start + 1]);

        if (num == 0xFFFF)
        {
            invalid_value++;
        }
    }

    invalid_value = package_points - invalid_value;
    if (invalid_value <= 1)
    {
        return;
    }

    for (std::int32_t i = 0; i < package_points; i++)
    {
        const auto s = packet_bytes[i * point_len + data_bits_start];
        const auto z = packet_bytes[i * point_len + data_bits_start + 1];

        const auto num = reinterpret(s, z);
        if (num != 0xFFFF)
        {
            scan_points_.emplace_back();
            scan_points_.back().range = num / 1000.;
            scan_points_.back().intensity = 0.;

            if ((degree + (degree_interval / invalid_value * i)) > 360.)
            {
                scan_points_.back().degree = degree + (degree_interval / invalid_value * i) - 360.;
            }
            else
            {
                scan_points_.back().degree = degree + (degree_interval / invalid_value * i);
            }
        }
    }
}

bool CLsLidar::impl::polling()
{
    if (!is_start)
    {
        return false;
    }

    scan_points_.clear();
    std::vector<std::byte> packet_bytes;
    bool difop = false;

    std::size_t len = 0;
    while (len == 0)
    {
        packet_bytes.clear();
        len = receive_data(packet_bytes);
        difop = compensation && (packet_bytes.size() > 187) &&
            ((packet_bytes[2] == std::byte{0x55}) &&
            (packet_bytes[3] == std::byte{0x00}) &&
            (packet_bytes[186] == std::byte{0xFA}) &&
            (packet_bytes[187] == std::byte{0xFB}));
    }

    if (difop)
    {
        difop_processing(packet_bytes);
    }
    else
    {
        data_processing(packet_bytes);
    }

    return true;
}

CLsLidar::CLsLidar()
    : m_impl(std::make_unique<impl>())
{
}

CLsLidar::~CLsLidar() = default;

bool CLsLidar::open(const std::string& port)
{
    return m_impl->open_serial(port);
}

bool CLsLidar::init()
{
    return m_impl->init();
}

bool CLsLidar::polling()
{
    return m_impl->polling();
}

const std::vector<ScanPoint>& CLsLidar::get() const
{
    return m_impl->scan_points_;
}