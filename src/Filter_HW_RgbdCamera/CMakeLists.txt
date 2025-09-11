# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_RgbdCamera)

add_library(${FILTER_NAME} SHARED
    src/CRgbdCameraFilter.h
    src/CRgbdCameraFilter.cpp
    src/CRgbdUtils.h
    src/CRgbdUtils.cpp
    src/api/unity/CRgbdCameraUnityApi.cpp
    src/api/unity/CRgbdCameraUnityApi.h
    #${OPENNI2_FILES}
)

target_include_directories(${FILTER_NAME}
    PUBLIC
        ${CMAKE_SOURCE_DIR}/CyC_Robots/src/RealSense/src/
        ${CMAKE_SOURCE_DIR}/CyC_Robots/src/RealSense/src/realsense-2.50.0/include)
       
target_link_directories(${FILTER_NAME}
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

# Check for Raspberry Pi platform
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")
    set(RASPBERRY_PI TRUE)
endif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")

target_link_libraries(${FILTER_NAME}
    PUBLIC
        enet
        Robot_RealSense
        CycCore
        CycToolkit
        config++
        ${CyC_OPENCV_WORLD})

if (WIN32)
    target_link_libraries(${FILTER_NAME}
        PUBLIC
            ws2_32
            winmm)
endif()

if (MSVC)
    target_link_libraries(${FILTER_NAME}
        PRIVATE
            #${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/win-msvc-x64/Lib/OpenNI2.lib
            realsense2)
elseif (RASPBERRY_PI)
    target_link_libraries(${FILTER_NAME}
        PRIVATE
            #${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/linux-gcc-arm-x86/lib/libOpenNI2.a
            realsense2
            realsense-file
            fmt
            fw
            usb-1.0)
else (RASPBERRY_PI)
    target_link_libraries(${FILTER_NAME}
        PRIVATE
            #${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/linux-gcc-x64/Redist/libOpenNI2.so
            realsense2
            realsense-file
            fmt
            fw
            usb-1.0)
endif(MSVC)

# Build the filter into the filters folder
set_target_properties(${FILTER_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
)
# Override config-specific paths to keep everything in the same folder
set_target_properties(${FILTER_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"

    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"

    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
)


# Test units
add_executable(tu_RgbdCamera src/tu_RgbdCamera.cpp)

target_include_directories(tu_RgbdCamera
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/cyc_robots/RealSense/src/
        ${Qt5Widgets_INCLUDE_DIRS}
        ${CyC_THIRD_PARTY}/glut-2.0.0.29/include
        ${CyC_THIRD_PARTY}/qt-5.12.2/win-msvc141-x64/include
        ${CMAKE_BINARY_DIR}/src/cyc_filters/Filters_Visualization/Filter_Visualization_Octoviz
        ${CMAKE_SOURCE_DIR}/src/cyc_filters/Filters_Visualization/Filter_Visualization_Octoviz/src
        ${CMAKE_SOURCE_DIR}/src/cyc_toolkit/src
        ${CMAKE_SOURCE_DIR}/src/cyc_filters/Filters_Visualization/Filter_Visualization_Octoviz/src/octoviz/include
        ${CMAKE_SOURCE_DIR}/src/cyc_filters/Filters_Visualization/Filter_Visualization_Octoviz/src/octoviz/src/extern/QGLViewer)

target_link_directories(tu_RgbdCamera
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(tu_RgbdCamera
    PUBLIC
        #${FILTER_NAME}
        #Filter_Visualization_Octoviz
        CycToolkit
        config++
        ${CyC_OPENCV_WORLD}
        octomap
        octomath
        #glut64
        Robot_RealSense
        fmt)

if (NOT WIN32)
    target_link_libraries(tu_RgbdCamera PUBLIC pthread)
endif()

add_executable(tu_RgbdCamera_DepthEncoding
    src/tu_RgbdCamera_DepthEncoding.cpp)

target_link_directories(tu_RgbdCamera_DepthEncoding
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(tu_RgbdCamera_DepthEncoding
    PUBLIC
        CycToolkit
        config++
        ${CyC_OPENCV_WORLD}
        fmt)
