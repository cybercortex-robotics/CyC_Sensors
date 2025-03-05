# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_Gps)

if (NOT Build_Filter_HW_Imu AND NOT Build_Filter_HW_Compass)
    SET(EXTRA_COMPILING_FILES src/interfaces/gps_bricklet_api/ip_connection.c)
endif (NOT Build_Filter_HW_Imu AND NOT Build_Filter_HW_Compass)

if (WIN32)
	SET(EXTRA_UBLOX_SRC_FILES src/interfaces/ublox_gps/Stream_win32.cpp)
endif (WIN32)

add_library(${FILTER_NAME}
	${CMAKE_SOURCE_DIR}/3rdParty/rs232/rs232.h 
	${CMAKE_SOURCE_DIR}/3rdParty/rs232/rs232.c
	src/CGpsFilter.h
	src/CGpsFilter.cpp
	src/interfaces/gps_bricklet_api/bricklet_gps_v2.h
	src/interfaces/gps_bricklet_api/bricklet_gps_v2.c
	src/interfaces/gps_bricklet_api/gps.h
	src/interfaces/gps_bricklet_api/gps.cpp
	src/interfaces/gps_bricklet_api/ip_connection.h
	${EXTRA_COMPILING_FILES}
	src/interfaces/ublox_gps/UbloxGPS.cpp
	src/interfaces/ublox_gps/Print.cpp
	src/interfaces/ublox_gps/Utils.cpp
	src/interfaces/ublox_gps/SparkFun_u-blox_GNSS_Arduino_Library.cpp
	${EXTRA_UBLOX_SRC_FILES})

target_include_directories(${FILTER_NAME}
	PUBLIC
		${CMAKE_SOURCE_DIR}/3rdParty/rs232
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/ublox_gps)
