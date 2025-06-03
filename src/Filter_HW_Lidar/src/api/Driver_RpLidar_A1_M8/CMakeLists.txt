# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

cmake_minimum_required(VERSION 3.12)
project(RpLidarAPI)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_guard()

set(HAL_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/hal/abs_rxtx.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/assert.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/byteops.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/event.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/locker.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/socket.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/thread.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/types.h
	${CMAKE_CURRENT_SOURCE_DIR}/hal/util.h)
	
if(WIN32)
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/win32)
	set(HAL_LIB_SRC ${HAL_LIB_SRC} 
	${CMAKE_CURRENT_SOURCE_DIR}/win32/arch_win32.h
	${CMAKE_CURRENT_SOURCE_DIR}/win32/net_serial.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/win32/net_serial.h
	${CMAKE_CURRENT_SOURCE_DIR}/win32/net_socket.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/win32/timer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/win32/timer.h
	${CMAKE_CURRENT_SOURCE_DIR}/win32/winthread.hpp
	)
else(WIN32)
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/linux)
	set(HAL_LIB_SRC ${HAL_LIB_SRC} 
	${CMAKE_CURRENT_SOURCE_DIR}/linux/arch_linux.h
	${CMAKE_CURRENT_SOURCE_DIR}/linux/net_serial.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/linux/net_serial.h
	${CMAKE_CURRENT_SOURCE_DIR}/linux/net_socket.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/linux/thread.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/linux/timer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/linux/timer.h
	)
endif(WIN32)

set(LIDAR_LIB_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/CRpLidarApi.h
    ${CMAKE_CURRENT_SOURCE_DIR}/CRpLidarApi.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar.h
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar_cmd.h 
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar_driver.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar_driver.h
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar_driver_impl.h
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar_driver_serial.h
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar_driver_TCP.h
	${CMAKE_CURRENT_SOURCE_DIR}/rplidar_protocol.h
	${CMAKE_CURRENT_SOURCE_DIR}/rptypes.h
	${CMAKE_CURRENT_SOURCE_DIR}/sdkcommon.h
	${CMAKE_CURRENT_SOURCE_DIR}/thread.cpp
	)

add_library(RpLidar_lib STATIC ${LIDAR_LIB_SRC} ${HAL_LIB_SRC})

set_target_properties(RpLidar_lib PROPERTIES LINKER_LANGUAGE CXX)