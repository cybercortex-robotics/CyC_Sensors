#ifdef WIN32
#include "serial_win.hpp"
#include <Windows.h>
//#include <iostream> // TODO: spdlog

namespace serial
{

Handle::Handle()
    : fd(INVALID_HANDLE_VALUE)
{
}

void Handle::replace(void* fd_)
{
    close();
    fd = fd_;
}

void Handle::close()
{
    if (fd != INVALID_HANDLE_VALUE)
    {
        if (CloseHandle(fd))
        {
            fd = INVALID_HANDLE_VALUE;
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
    handle.replace(CreateFileA(port.data(),
        GENERIC_READ | GENERIC_WRITE,
        0,                          /* no share  */
        NULL,                       /* no security */
        OPEN_EXISTING,
        0,                          /* no threads */
        NULL));                     /* no templates */

    if (handle.get() == INVALID_HANDLE_VALUE)
    {
        spdlog::error("{}: unable to open comport '{}'", typeid(*this).name(), port);
        return false;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState((HANDLE)handle.get(), &dcbSerialParams))
    {
        //spdlog::error("Error getting COM state");
        return false;
    }

    dcbSerialParams.BaudRate = baud_rate;  // Set baud rate
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    dcbSerialParams.fOutxCtsFlow = FALSE; // Disable CTS flow control

    // Set data bits
    switch (data_bits)
    {
    case DataBits::bits_7:
        dcbSerialParams.ByteSize = 7;
        break;
    case DataBits::bits_8:
    default:
        dcbSerialParams.ByteSize = 8;
        break;
    }

    switch (parity)
    {
    case Parity::odd:
        dcbSerialParams.Parity = ODDPARITY;
        break;
    case Parity::even:
        dcbSerialParams.Parity = EVENPARITY;
        break;
    case Parity::none:
    default:
        dcbSerialParams.Parity = NOPARITY;
        break;
    }

    switch (stop_bits)
    {
    case StopBits::bits_1:
    default:
        dcbSerialParams.StopBits = ONESTOPBIT;
        break;
    case StopBits::bits_2:
        dcbSerialParams.StopBits = TWOSTOPBITS;
        break;
    }

    if (!SetCommState((HANDLE)handle.get(), &dcbSerialParams))
    {
        //spdlog::error("Error setting COM state");
        return false;
    }

    COMMTIMEOUTS timeouts = { 0 };
    GetCommTimeouts((HANDLE)handle.get(), &timeouts);

    timeouts.ReadIntervalTimeout = 100;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.ReadTotalTimeoutConstant = 100;

    if (!SetCommTimeouts((HANDLE)handle.get(), &timeouts))
    {
        //spdlog::error("Error setting timeouts");
        return false;
    }

    return true;
}

void Serial::impl::close()
{
    handle.close();
}

std::int64_t Serial::impl::read_some_into(std::byte* buf, std::size_t available_length, std::chrono::milliseconds timeout) const
{
    if (handle.get() == INVALID_HANDLE_VALUE)
    {
        return -1;
    }

    memset(buf, 0, available_length);

    std::int64_t totalBytesRead = 0;
    std::int64_t rc = 0;
    DWORD read_size = 0;
    std::int64_t unlink = 0;

    if (timeout.count() > 0)
    {
        rc = wait_readable(timeout);
        if (rc <= 0)
        {
            return (rc == 0) ? 0 : -1;
        }

        int	retry = 3;
        while (available_length > 0)
        {
            auto const result = ReadFile(handle.get(), buf, available_length, &read_size, NULL);
            if (read_size > 0)
            {
                available_length -= read_size;
                buf += read_size;
                totalBytesRead += read_size;

                if (available_length == 0)
                {
                    break;
                }
            }
            else if (!result)
            {
                retry--;
                if (retry <= 0)
                {
                    break;
                }
            }

            unlink++;
            rc = wait_readable(20ms);
            if (unlink > 10)
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
        auto const result = ReadFile(handle.get(), buf, available_length, &read_size, NULL);
        if (read_size > 0)
        {
            totalBytesRead += read_size;
        }
        else if (!result && (GetLastError() != ERROR_IO_PENDING))
        {
            return -1;
        }
    }

    return totalBytesRead;
}

std::int64_t Serial::impl::wait_readable(std::chrono::milliseconds timeout) const
{
    return wait_flag(timeout, EV_RXCHAR);
}

std::int64_t Serial::impl::write_some(std::byte const* buf, std::size_t available_length, std::chrono::milliseconds timeout) const
{
    if (handle.get() == INVALID_HANDLE_VALUE)
    {
        return -1;
    }

    if (available_length == 0)
    {
        return 0;
    }

    std::int64_t totalBytesWrite = 0;
    std::int64_t rc = 0;
    DWORD write_size = 0;

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
            auto const result = WriteFile(handle.get(), buf, available_length, &write_size, NULL);
            if (result && (write_size > 0))
            {
                available_length -= write_size;
                buf += write_size;
                totalBytesWrite += write_size;

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
        auto const result = WriteFile(handle.get(), buf, available_length, &write_size, NULL);
        if (result && (write_size > 0))
        {
            totalBytesWrite += write_size;
        }
        else if (!result && (GetLastError() != ERROR_IO_PENDING))
        {
            return -1;
        }
    }

    return totalBytesWrite;
}

std::int64_t Serial::impl::wait_writable(std::chrono::milliseconds timeout) const
{
    return 1;

    // Does not work. For some reason, this hangs, but just writing to the serial works anyway.
    // return wait_flag(timeout, EV_TXEMPTY);
}

std::int64_t Serial::impl::wait_flag(std::chrono::milliseconds timeout, std::uint32_t flag) const
{
    auto const hSerial = (HANDLE)handle.get();
    DWORD eventMask{};

    auto millis = timeout.count();
    while (millis > 0)
    {
        auto const waitTime = (millis < 5000) ? millis : 5000;
        millis -= waitTime;

        if (!SetCommMask(hSerial, flag))  // Set mask for received character event
        {
            //spdlog::error("Error setting comm mask");
            return -1;
        }

        if (WaitCommEvent(hSerial, &eventMask, NULL))
        {
            if (eventMask & flag)
            {
                return 1;  // Data is available to read
            }
        }
        else
        {
            auto const error = GetLastError();
            if (error != ERROR_IO_PENDING)
            {
                //spdlog::error("Error waiting for event: {}", eventMask);
                return -1;
            }
        }
    }

    return 0;
}

} // namespace serial

#endif // WIN32
