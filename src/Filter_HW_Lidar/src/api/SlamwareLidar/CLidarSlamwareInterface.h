#ifndef ROVIS_LIDAR_SLAMWARE_INTERFACE_H_
#define ROVIS_LIDAR_SLAMWARE_INTERFACE_H_

#include <boost/asio.hpp>
#include <chrono>

namespace SeqPacket {
    struct seqpacket_protocol
    {
        int type()     const { return SOCK_SEQPACKET; }
        int protocol() const { return 0;            }
        int family()   const { return AF_UNIX;      }

        typedef boost::asio::local::basic_endpoint<seqpacket_protocol> endpoint;
        typedef boost::asio::basic_seq_packet_socket<seqpacket_protocol> socket;
        typedef boost::asio::basic_socket_acceptor<seqpacket_protocol> acceptor;

#if !defined(BOOST_ASIO_NO_IOSTREAM)
        /// The UNIX domain iostream type.
        typedef boost::asio::basic_socket_iostream<seqpacket_protocol> iostream;
#endif // !defined(BOOST_ASIO_NO_IOSTREAM)
    };
}

struct lidar_header
{
    std::chrono::milliseconds::rep timestamp = 0;
    size_t num_points = 0;
};

struct lidar_point
{
    float distance;
    float angle;
    bool valid;
};

class CLidarSlamwareInterface
{
public:
    CLidarSlamwareInterface();
    CLidarSlamwareInterface(const CLidarSlamwareInterface&) = delete;
    CLidarSlamwareInterface(CLidarSlamwareInterface&&) = delete;
    CLidarSlamwareInterface& operator=(const CLidarSlamwareInterface&) = delete;
    CLidarSlamwareInterface& operator=(CLidarSlamwareInterface&&) = delete;
    ~CLidarSlamwareInterface() = default;

    bool connect();
    bool process(std::vector<lidar_point>& points);

private:
    const char* socket_name = "/tmp/rovis_mapper_socket";

    boost::asio::io_service io_service;
    SeqPacket::seqpacket_protocol::socket uds;
};

#endif // ROVIS_LIDAR_SLAMWARE_INTERFACE_H_
