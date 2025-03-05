// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "sdkcommon.h"
#include "hal/thread.h"

#if defined(_WIN32)
#include "winthread.hpp"
#elif defined(_MACOS)
#include "arch/macOS/thread.hpp"
#elif defined(__GNUC__)
#include "arch/linux/thread.hpp"
#else
#error no threading implemention found for this platform.
#endif


