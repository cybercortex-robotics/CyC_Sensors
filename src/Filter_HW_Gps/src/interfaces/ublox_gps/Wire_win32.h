#ifndef Wire_h
#define Wire_h

#include "Common.h"

class TwoWire
{
public:
    TwoWire() = default;
    TwoWire(uint8_t default_address) {}
    ~TwoWire() = default;

    bool init(const char* filename = I2C_DEV)
    {
        throw std::exception{ "Wire is not implemented." };
    }

    bool beginTransmission(uint8_t address)
    {
        throw std::exception{ "Wire is not implemented." };
    }

    uint8_t endTransmission(void)
    {
        throw std::exception{ "Wire is not implemented." };
    }

    uint8_t endTransmission(bool stop) { return endTransmission(); }

    bool available()
    {
        throw std::exception{ "Wire is not implemented." };
    }

    uint8_t read()
    {
        throw std::exception{ "Wire is not implemented." };
    }

    size_t write(uint8_t byte)
    {
        throw std::exception{ "Wire is not implemented." };
    }

    size_t write(uint8_t* byte, size_t num_of_bytes)
    {
        throw std::exception{ "Wire is not implemented." };
    }

    uint8_t get_available_bytes()
    {
        throw std::exception{ "Wire is not implemented." };
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity)
    {
        throw std::exception{ "Wire is not implemented." };
    }
};

extern TwoWire Wire;

#endif //Wire_h
