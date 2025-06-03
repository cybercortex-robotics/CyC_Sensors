#include "serial.hpp"
#include "serial_unix.hpp"
#include "serial_win.hpp"

namespace serial
{

Serial::Serial()
    : m_impl(std::make_unique<impl>())
{
}

Serial::~Serial() = default;


bool Serial::open(std::string_view port, std::uint32_t baud_rate, DataBits data_bits, Parity parity, StopBits stop_bits)
{
    return m_impl->open(port, baud_rate, data_bits, parity, stop_bits);
}

std::vector<std::byte> Serial::read_exact(std::int64_t length, std::chrono::milliseconds timeout) const
{
    std::vector<std::byte> buf;
    append_read_exact_into(buf, length, timeout);

    return buf;
}

std::vector<std::byte> Serial::read_some(std::int64_t length, std::chrono::milliseconds timeout) const
{
    std::vector<std::byte> buf;
    append_read_some_into(buf, length, timeout);

    return buf;
}

bool Serial::read_exact_into(std::byte* buf, std::size_t length, std::chrono::milliseconds timeout) const
{
    std::int64_t total_read = 0;
    while (total_read < length)
    {
        auto const read_size = read_some_into(std::next(buf, total_read), length - total_read, timeout);
        if (read_size < 0)
        {
            return false;
        }

        total_read += read_size;
    }

    return total_read == length;
}

std::int64_t Serial::read_some_into(std::byte* buf, std::size_t length, std::chrono::milliseconds timeout) const
{
    return m_impl->read_some_into(buf, length, timeout);
}

bool Serial::append_read_exact_into(std::vector<std::byte>& vec, std::size_t length, std::chrono::milliseconds timeout) const
{
    const auto previous_length = vec.size();
    vec.resize(previous_length + length);
    std::byte* const buf = std::next(vec.data(), previous_length);

    if (!read_exact_into(buf, length, timeout))
    {
        vec.resize(previous_length);
        return false;
    }

    return true;
}

std::int64_t Serial::append_read_some_into(std::vector<std::byte>& vec, std::size_t max_length, std::chrono::milliseconds timeout) const
{
    auto const previous_length = vec.size();
    vec.resize(previous_length + max_length);
    std::byte* const buf = std::next(vec.data(), previous_length);

    const auto rc = read_some_into(buf, max_length, timeout);
    if (rc < 0)
    {
        vec.resize(previous_length);
    }
    else
    {
        vec.resize(previous_length + rc);
    }

    return rc;
}

bool Serial::replace_read_exact_into(std::vector<std::byte>& vec, std::size_t new_length, std::chrono::milliseconds timeout) const
{
    vec.clear();
    return append_read_exact_into(vec, new_length, timeout);
}

std::int64_t Serial::replace_read_some_into(std::vector<std::byte>& vec, std::size_t max_length, std::chrono::milliseconds timeout) const
{
    vec.clear();
    return append_read_some_into(vec, max_length, timeout);
}

bool Serial::write_exact(std::vector<std::byte> const& vec, std::chrono::milliseconds timeout) const
{
    return write_exact(vec, 0, timeout);
}

bool Serial::write_exact(std::vector<std::byte> const& vec, std::size_t offset, std::chrono::milliseconds timeout) const
{
    std::int64_t total_write = offset;
    while (total_write < vec.size())
    {
        auto const write_size = write_some(vec, total_write, timeout);
        if (write_size < 0)
        {
            return false;
        }

        total_write += write_size;
    }

    return true;
}

std::int64_t Serial::write_some(std::vector<std::byte> const& vec, std::chrono::milliseconds timeout) const
{
    return write_some(vec, 0, timeout);
}

std::int64_t Serial::write_some(std::vector<std::byte> const& vec, std::size_t offset, std::chrono::milliseconds timeout) const
{
    if (offset >= vec.size())
    {
        return 0; // nothing to write
    }

    return m_impl->write_some(std::next(vec.data(), offset), vec.size() - offset, timeout);
}


} // namespace serial
