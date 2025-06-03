# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_Gps)

#if (NOT Build_Filter_HW_Imu AND NOT Build_Filter_HW_Compass)
#    SET(EXTRA_COMPILING_FILES src/interfaces/gps_bricklet_api/ip_connection.c)
#endif (NOT Build_Filter_HW_Imu AND NOT Build_Filter_HW_Compass)

if (WIN32)
	SET(EXTRA_UBLOX_SRC_FILES src/interfaces/ublox_gps/Stream_win32.cpp)
endif (WIN32)

add_library(${FILTER_NAME} SHARED
	${CyC_THIRD_PARTY}/rs232/rs232.h 
	${CyC_THIRD_PARTY}/rs232/rs232.c
	src/CGpsFilter.h
	src/CGpsFilter.cpp
#	src/interfaces/gps_bricklet_api/bricklet_gps_v2.h
#	src/interfaces/gps_bricklet_api/bricklet_gps_v2.c
#	src/interfaces/gps_bricklet_api/gps.h
#	src/interfaces/gps_bricklet_api/gps.cpp
#	src/interfaces/gps_bricklet_api/ip_connection.h
#	${EXTRA_COMPILING_FILES}
	src/interfaces/ublox_gps/UbloxGPS.cpp
	src/interfaces/ublox_gps/Print.cpp
	src/interfaces/ublox_gps/Utils.cpp
	src/interfaces/ublox_gps/SparkFun_u-blox_GNSS_Arduino_Library.cpp
	${EXTRA_UBLOX_SRC_FILES})

target_include_directories(${FILTER_NAME}
	PUBLIC
		${CyC_THIRD_PARTY}/rs232
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/ublox_gps)

target_link_directories(${FILTER_NAME}
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})
        
target_link_libraries(${FILTER_NAME}
    PUBLIC
        CycCore
        CycToolkit
        libconfig++
        libcsv
        ${CyC_OPENCV_WORLD})

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
