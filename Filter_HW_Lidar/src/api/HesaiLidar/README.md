# HesaiLidar_General_SDK

## Environment and Dependencies
**System environment requirement: Linux + G++ 7.0 or above**
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**
```
$ sudo apt install libpcap-dev libyaml-cpp-dev
$ sudo apt install libpcl-dev
```

## Add to the project
### Cmake
```
add_subdirectory(<path_to>HesaiLidar_General_SDK)

include_directories(
	<path_to>HesaiLidar_General_SDK/include
	<path_to>HesaiLidar_General_SDK/src/PandarGeneralRaw/include
)

```
### C++
```
#include "include/pandarGeneral_sdk.h"
// for Pandar40P
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar40P"));
```
