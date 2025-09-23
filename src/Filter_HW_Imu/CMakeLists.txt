# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_Imu)

add_library(${FILTER_NAME} SHARED
    ${CyC_THIRD_PARTY}/rs232/rs232.h 
    ${CyC_THIRD_PARTY}/rs232/rs232.c
    src/CImuUtils.h
    src/CImuUtils.cpp
    src/CImuFilter.h
    src/CImuFilter.cpp
    src/CImuSim.h
    src/CImuSim.cpp
    src/api/MPU9250/CImuMPU9250_API.h
    src/api/MPU9250/CImuMPU9250_API.cpp
    src/api/brick/CImuBrick_API.h
    src/api/brick/CImuBrick_API.cpp
    src/api/brick/brick_imu_v2.h
    src/api/brick/brick_imu_v2.cpp
    src/api/brick/ip_connection.h
    src/api/brick/ip_connection.cpp
    src/api/android/CNativeSensor_API.cpp
    src/api/android/CNativeSensor_API.h
    src/api/unity/CImuUnity_API.cpp
    src/api/unity/CImuUnity_API.h)
    
target_include_directories(${FILTER_NAME}
    PUBLIC
        ${CyC_THIRD_PARTY}/rs232)
        
target_link_directories(${FILTER_NAME}
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(${FILTER_NAME}
    PUBLIC
        enet
        CycCore
        CycToolkit
        ${CyC_OPENCV_WORLD}
        Robot_RealSense)

if (WIN32)
    target_link_libraries(${FILTER_NAME} PUBLIC libconfig++ ws2_32 winmm)
else ()
    target_link_libraries(${FILTER_NAME} PUBLIC config++)
endif()

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
add_executable(tu_Imu
    src/tu_Imu.cpp
    src/api/MPU9250/CImuMPU9250_API.h
    src/api/MPU9250/CImuMPU9250_API.cpp
    ${CyC_THIRD_PARTY}/rs232/rs232.h 
    ${CyC_THIRD_PARTY}/rs232/rs232.c)

target_link_directories(tu_Imu
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(tu_Imu
    PUBLIC
        ${FILTER_NAME}
        CycToolkit
        ${CyC_OPENCV_WORLD}
        Robot_RealSense
        fmt)

if (Build_Robot_UnitreeA1)
    add_definitions(-DENABLE_UNITREEA1)
    target_link_libraries(${FILTER_NAME}
        PRIVATE
            Robot_UnitreeA1)
endif (Build_Robot_UnitreeA1)
