#pragma once

#ifndef WIN32
#include "serial.hpp"

namespace serial
{

struct Handle
{
    Handle();
    ~Handle();

    void replace(int fd_);
    int release();
    
    void close();

    int get() const
    {
        return fd;
    }

private:
    int fd = -1;
};

struct Serial::impl
{
    impl();
    ~impl();

    bool open(std::string_view port, std::uint32_t baud_rate, DataBits data_bits, Parity parity, StopBits stop_bits);
    void close();

    std::int64_t read_some_into(std::byte* buf, std::size_t available_length, std::chrono::milliseconds timeout) const;
    std::int64_t wait_readable(std::chrono::milliseconds timeout) const;

    std::int64_t write_some(std::byte const* buf, std::size_t available_length, std::chrono::milliseconds timeout) const;
    std::int64_t wait_writable(std::chrono::milliseconds timeout) const;

    Handle handle;
};

} // namespace serial

#endif // WIN32
