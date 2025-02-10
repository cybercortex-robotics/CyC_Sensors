# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_MonoCamera)

if(RASPBERRY_PI)
    add_library(${FILTER_NAME}
        include/CMonoCameraFilter.h
        src/CMonoCameraFilter.cpp
        src/interfaces/raspi/CMonoCameraRaspiInterface.cpp
        src/interfaces/raspi/CMonoCameraRaspiInterface.h
        src/interfaces/raspi/raspicam.h
        src/interfaces/raspi/raspicam_cv.h
        src/interfaces/raspi/raspicamtypes.h)

    target_link_libraries(${FILTER_NAME}
        PUBLIC
            CcrToolkit
            raspicam
            raspicam_cv)
else(RASPBERRY_PI)
    add_library(${FILTER_NAME}
        include/CMonoCameraFilter.h
        src/CMonoCameraFilter.cpp
        src/interfaces/unity/CMonoCameraUnityInterface.cpp
        src/interfaces/unity/CMonoCameraUnityInterface.h
        src/interfaces/android/CNativeCameraInterface.cpp
        src/interfaces/android/CNativeCameraInterface.h)

    target_include_directories(${FILTER_NAME}
        PUBLIC
            ${CMAKE_SOURCE_DIR}/src/ccr_robots/ARParotDrone/src)

    target_link_libraries(${FILTER_NAME}
        PRIVATE
            CcrToolkit
            enet)
endif(RASPBERRY_PI)

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
