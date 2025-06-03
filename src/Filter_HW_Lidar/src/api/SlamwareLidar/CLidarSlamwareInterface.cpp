#include "CLidarSlamwareInterface.h"
#include <spdlog/spdlog-inl.h>

CLidarSlamwareInterface::CLidarSlamwareInterface()
    : uds(io_service)
{
}

bool CLidarSlamwareInterface::connect()
{
    try
    {
        uds.connect(socket_name);
    }
    catch (boost::system::system_error& e)
    {
        spdlog::error("CLidarSlamwareInterface: {}", e.what());
        return false;
    }
    catch (...)
    {
        spdlog::error("CLidarSlamwareInterface: failed to connect to Slamware lidar");
        return false;
    }

    spdlog::info("Connection successfully");
    return true;
}

bool CLidarSlamwareInterface::process(std::vector<lidar_point>& points)
{
    try
    {
        lidar_header header{};
        boost::asio::socket_base::message_flags out_flags{};
        uds.receive(boost::asio::buffer(&header, sizeof(header)), out_flags);

        points.resize(header.num_points);

        const auto size_in_bytes = points.size() * sizeof(lidar_point);
        uds.receive(boost::asio::buffer(points.data(), size_in_bytes), out_flags);
    }
    catch (...)
    {
        return false;
    }

    return true;
}
