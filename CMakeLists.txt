# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

cmake_minimum_required(VERSION 3.10)
project(CyC_HW_Sensors)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(inference)

# Hardware sensory filters
add_ccr_filter(Filter_HW_MonoCamera Filter_HW_MonoCamera)
