# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_MonoCamera)

if(RASPBERRY_PI)
    add_library(${FILTER_NAME} SHARED
        src/CMonoCameraFilter.h
        src/CMonoCameraFilter.cpp
        src/interfaces/raspi/CMonoCameraRaspiInterface.cpp
        src/interfaces/raspi/CMonoCameraRaspiInterface.h
        src/interfaces/raspi/raspicam.h
        src/interfaces/raspi/raspicam_cv.h
        src/interfaces/raspi/raspicamtypes.h)
    target_link_directories(${FILTER_NAME}
        PUBLIC 
            ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})
    target_link_libraries(${FILTER_NAME}
        PUBLIC
            raspicam
            raspicam_cv)
else(RASPBERRY_PI)
    add_library(${FILTER_NAME} SHARED
        src/CMonoCameraFilter.h
        src/CMonoCameraFilter.cpp
        src/interfaces/unity/CMonoCameraUnityInterface.cpp
        src/interfaces/unity/CMonoCameraUnityInterface.h
        src/interfaces/android/CNativeCameraInterface.cpp
        src/interfaces/android/CNativeCameraInterface.h)
    target_include_directories(${FILTER_NAME}
        PUBLIC
            ${CMAKE_SOURCE_DIR}/src/CyC_robots/ARParotDrone/src)
    target_link_directories(${FILTER_NAME}
        PUBLIC 
            ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})
endif(RASPBERRY_PI)

target_link_libraries(${FILTER_NAME}
    PUBLIC
        enet
        config++
        fmt
        CycCore
        CycToolkit
        ${CyC_OPENCV_WORLD})

if (WIN32)
    target_link_libraries(${FILTER_NAME}
        PUBLIC
            ws2_32
            winmm)
endif()

if (Build_Robot_ARParotDrone)
    add_definitions(-DROBOT_ARPAROTDRONE_ENABLED)
    target_link_libraries(${FILTER_NAME}
        PRIVATE
            Robot_ARParotDrone)
endif (Build_Robot_ARParotDrone)

if (Build_Robot_ParrotANAFI)
    add_definitions(-DROBOT_PARROTANAFI_ENABLED)
    target_link_libraries(${FILTER_NAME}
        PRIVATE
            Robot_ParrotANAFI)
endif (Build_Robot_ParrotANAFI)

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
