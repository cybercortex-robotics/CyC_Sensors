# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

cmake_minimum_required(VERSION 3.10)
project(CyC_HW_Sensors)
set(CMAKE_CXX_STANDARD 17)

# Folder for building the filters DLLs
set(CCR_FILTERS_PATH "filters")

# Folder with 3rdParty libraries
set(CCR_THIRD_PARTY "C:/dev/src/CyberCortex.AI/core/3rdParty")

# Add CyberCortex inference engine
add_subdirectory(inference)

# Include 3rdParty directories
include_directories(${CCR_THIRD_PARTY_INCLUDE_DIRECTORIES})

include_directories(inference/include)

# Include 3rdParty library directories
link_directories(${CCR_THIRD_PARTY_LIBRARY_DIRECTORIES})

# Hardware sensory filters
add_ccr_filter(Filter_HW_MonoCamera Filter_HW_MonoCamera)
