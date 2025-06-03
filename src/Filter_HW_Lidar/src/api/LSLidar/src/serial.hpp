#pragma once

#include "CyC_TYPES.h"
#include <string_view>
#include <memory>
#include <vector>
#include <cstddef>
#include <chrono>

namespace serial {

using namespace std::chrono_literals;

enum class DataBits
{
    bits_7,
    bits_8
};

enum class Parity
{
    odd,
    even,
    none
};

enum class StopBits
{
    bits_1,
    bits_2,
};

class Serial
{
public:
    Serial();
    Serial(const Serial&) = delete;
    Serial(Serial&&) = default;
    Serial& operator=(const Serial&) = delete;
    Serial& operator=(Serial&&) = default;
    ~Serial();

    bool open(std::string_view port, std::uint32_t baud_rate, DataBits data_bits, Parity parity, StopBits stop_bits);

    std::vector<std::byte> read_exact(std::int64_t length, std::chrono::milliseconds timeout = 30ms) const;
    std::vector<std::byte> read_some(std::int64_t max_length, std::chrono::milliseconds timeout = 30ms) const;
    bool read_exact_into(std::byte* buf, std::size_t length, std::chrono::milliseconds timeout = 30ms) const;
    std::int64_t read_some_into(std::byte* buf, std::size_t length, std::chrono::milliseconds timeout = 30ms) const;

    bool append_read_exact_into(std::vector<std::byte>& vec, std::size_t length, std::chrono::milliseconds timeout = 30ms) const;
    std::int64_t append_read_some_into(std::vector<std::byte>& vec, std::size_t max_length, std::chrono::milliseconds timeout = 30ms) const;

    bool replace_read_exact_into(std::vector<std::byte>& vec, std::size_t new_length, std::chrono::milliseconds timeout = 30ms) const;
    std::int64_t replace_read_some_into(std::vector<std::byte>& vec, std::size_t max_length, std::chrono::milliseconds timeout = 30ms) const;

    bool write_exact(std::vector<std::byte> const& vec, std::chrono::milliseconds timeout = 30ms) const;
    bool write_exact(std::vector<std::byte> const& vec, std::size_t offset, std::chrono::milliseconds timeout = 30ms) const;
    std::int64_t write_some(std::vector<std::byte> const& vec, std::chrono::milliseconds timeout = 30ms) const;
    std::int64_t write_some(std::vector<std::byte> const& vec, std::size_t offset, std::chrono::milliseconds timeout = 30ms) const;

private:
    struct impl;
    std::unique_ptr<impl> m_impl;
};

} // namespace serial
