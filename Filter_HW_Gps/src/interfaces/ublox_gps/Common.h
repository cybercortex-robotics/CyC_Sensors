#ifndef COMMON_H__
#define COMMON_H__

#include <cstdint>

typedef uint8_t byte;

#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1

#define SHORT_BUFF 256

#define I2C_DEV "/dev/ublox_i2c"

#ifdef WIN32
#define __attribute__(ATTR) 
#endif

#endif // COMMON_H__