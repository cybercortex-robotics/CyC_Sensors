#ifndef WIN32
#include "serial_unix.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

//#include <iostream> // TODO: spdlog

namespace serial
{

Handle::Handle()
    : fd(-1)
{
}

void Handle::replace(int fd_)
{
    close();
    fd = fd_;
}

int Handle::release()
{
    auto old_fd = fd;
    fd = -1;

    return old_fd;
}

void Handle::close()
{
    if (fd != -1)
    {
        if (::close(fd))
        {
            fd = -1;
        }
    }
}

Handle::~Handle()
{
    close();
}

Serial::impl::impl() = default;

Serial::impl::~impl()
{
    close();
}

bool Serial::impl::open(std::string_view port, std::uint32_t baud_rate, DataBits data_bits, Parity parity, StopBits stop_bits)
{
    Handle handle_;
    handle_.replace(::open(port.data(), O_RDWR|O_NOCTTY|O_NDELAY));

    if (handle_.get() > 0)
    {
        struct termios newtio, oldtio;
        if (tcgetattr(handle_.get(), &oldtio) != 0)
        {
            return false;
        }

        bzero(&newtio, sizeof(newtio));

        newtio.c_cflag |= CLOCAL;
        newtio.c_cflag |= CREAD;

        switch (data_bits)
        {
        case DataBits::bits_7:
            newtio.c_cflag |= CS7;
            break;
        case DataBits::bits_8:
        default:
            newtio.c_cflag |= CS8;
            break;
        }

        switch (parity)
        {
        case Parity::odd:
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            break;
        case Parity::even:
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case Parity::none:
        default:
            newtio.c_cflag &= ~PARENB;
            break;
        }

        switch (baud_rate)
        {
        case 230400:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        case 500000:
            cfsetispeed(&newtio, B500000);
            cfsetospeed(&newtio, B500000);
            break;
        case 921600:
            cfsetispeed(&newtio, B921600);
            cfsetospeed(&newtio, B921600);
            break;
        default:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        }

        switch (stop_bits)
        {
        case StopBits::bits_1:
        default:
            newtio.c_cflag &= ~CSTOPB;
            break;
        case StopBits::bits_2:
            newtio.c_cflag |= CSTOPB;
            break;
        }

        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;

        tcflush(handle_.get(), TCIFLUSH);

        if ((tcsetattr(handle_.get(), TCSANOW, &newtio)) != 0)
        {
            return false;
        }

        handle.replace(handle_.release());
        return true;
    }

    return false;
}

void Serial::impl::close()
{
    handle.replace(-1);
}

std::int64_t Serial::impl::read_some_into(std::byte* buf, std::size_t available_length, std::chrono::milliseconds timeout) const
{
    if (handle.get() < 0)
    {
        return -1;
    }

    if ((buf == nullptr) || (available_length == 0))
    {
        return -1;
    }

    std::int64_t totalBytesRead = 0;
    std::int32_t rc;
    std::int32_t unlink = 0;

    if (timeout.count() > 0)
    {
        rc = wait_readable(timeout);
        if (rc <= 0)
        {
            return (rc == 0) ? 0 : -1;
        }

        std::int32_t retry = 3;
        while (available_length > 0)
        {
            rc = ::read(handle.get(), buf, available_length);

            if (rc > 0)
            {
                available_length -= rc;
                buf += rc;
                totalBytesRead += rc;

                if (available_length == 0)
                {
                    break;
                }
            }
            else if (rc < 0)
            {
                retry--;
                if (retry <= 0)
                {
                    break;
                }
            }

            unlink++;
            rc = wait_readable(20ms);
            if(unlink > 10)
            {
                return -1;
            }
            
            if (rc <= 0)
            {
                break;
            }
        }
    }
    else
    {
        rc = ::read(handle.get(), buf, available_length);
        if (rc > 0)
        {
            totalBytesRead += rc;
        }
        else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN))
        {
            return -1;
        }
    }
  
    return totalBytesRead;
}

std::int64_t Serial::impl::wait_readable(std::chrono::milliseconds timeout) const
{
    fd_set fdset;
    struct timeval tv;
    std::int32_t rc = 0;

    auto millis = timeout.count();
    while (millis > 0)
    {
        if (millis < 5000)
        {
            tv.tv_usec = millis % 1000 * 1000;
            tv.tv_sec  = millis / 1000;
    
            millis = 0;
        }
        else
        {
            tv.tv_usec = 0;
            tv.tv_sec  = 5;
    
            millis -= 5000;
        }
    
        FD_ZERO(&fdset);
        FD_SET(handle.get(), &fdset);
        
        rc = select(handle.get() + 1, &fdset, NULL, NULL, &tv);
        if (rc > 0)
        {
            rc = (FD_ISSET(handle.get(), &fdset)) ? 1 : -1;
            break;
        }
        else if (rc < 0)
        {
            rc = -1;
            break;
        }
    }
  
    return rc;
}

std::int64_t Serial::impl::write_some(std::byte const* buf, std::size_t available_length, std::chrono::milliseconds timeout) const
{
    if (handle.get() < 0)
    {
        return -1;
    }

    if ((buf == nullptr) || (available_length == 0))
    {
        return -1;
    }

    std::int32_t totalBytesWrite = 0;
    std::int32_t rc;

    if (timeout.count() > 0)
    {
        rc = wait_writable(timeout);
        if (rc <= 0)
        {
            return (rc == 0) ? 0 : -1;
        }

        int	retry = 3;
        while (available_length > 0)
        {
            rc = write(handle.get(), buf, available_length);
            if (rc > 0)
            {
                available_length -= rc;
                buf += rc;
                totalBytesWrite += rc;

                if (available_length == 0)
                {
                    break;
                }
            }
            else
            {
                retry--;
                if (retry <= 0)
                {
                    break;
                }
            }
    
            rc = wait_writable(50ms);
            if (rc <= 0)
            {
                break;
            }
        }
    }
    else
    {
        rc = write(handle.get(), buf, available_length);
        if (rc > 0)
        {
            totalBytesWrite += rc;
        }
        else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN))
        {
            return -1;
        }
    }
  
    return totalBytesWrite;
}

std::int64_t Serial::impl::wait_writable(std::chrono::milliseconds timeout) const
{
    if (handle.get() < 0)
    {
        return -1;
    }

    fd_set fdset;
    struct timeval tv;
    std::int32_t rc = 0;

    auto millis = timeout.count();
    while (millis > 0)
    {
        if (millis < 5000)
        {
            tv.tv_usec = millis % 1000 * 1000;
            tv.tv_sec  = millis / 1000;
    
            millis = 0;
        }
        else
        {
            tv.tv_usec = 0;
            tv.tv_sec  = 5;
    
            millis -= 5000;
        }
    
        FD_ZERO(&fdset);
        FD_SET(handle.get(), &fdset);
    
        rc = select(handle.get() + 1, NULL, &fdset, NULL, &tv);
        if (rc > 0)
        {
            rc = (FD_ISSET(handle.get(), &fdset)) ? 1 : -1;
            break;
        }
        else if (rc < 0)
        {
            rc = -1;
            break;
        }
    }
  
    return rc;
}

}

#endif // not WIN32
