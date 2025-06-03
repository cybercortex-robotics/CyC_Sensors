# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

cmake_minimum_required(VERSION 3.10)
project(CyC_HW_Sensors)
set(CMAKE_CXX_STANDARD 17)

######## USER DEFINED VARIABLES ########
# Folder for building the filters DLLs
set(CyC_FILTERS_PATH "filters")

# Folder with 3rdParty libraries
set(CyC_THIRD_PARTY "C:/dev/src/CyberCortex.AI/core/3rdParty")

# Define the OpenCV version
set(CyC_OPENCV_VERSION "4.9.0")

# Define the CUDA Toolkit SDK version
set(CyC_CUDA_VERSION "11.8")

# Define the libtorch and torchvision versions
set(CyC_TORCH_VERSION "2.1.2")
set(CyC_ONNXRUNTIME_VERSION "1.16.3")
######## !USER DEFINED VARIABLES ########

# Add CyberCortex inference engine
add_subdirectory(CyC_inference)

if (NOT ANDROID)
    #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}/bin>)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif (NOT ANDROID)

# Copy inference app
configure_file(
  ${CMAKE_SOURCE_DIR}/CyC_inference/bin/${CyC_LIBS_BINARY_KEY}/App_CycCore.exe
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/App_CycCore.exe
  COPYONLY
)

# Include 3rdParty directories
include_directories(${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES})

include_directories(inference/include)
include_directories(inference/include/core)

# Include 3rdParty library directories
link_directories(${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

# Build Robots
add_subdirectory(CyC_Robots/src)

# Hardware sensory filters
add_cyc_filter(Filter_HW_MonoCamera src/Filter_HW_MonoCamera)
add_cyc_filter(Filter_HW_RgbdCamera src/Filter_HW_RgbdCamera)
