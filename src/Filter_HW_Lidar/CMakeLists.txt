# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_Lidar)

add_subdirectory(src/api/LSLidar)

if (NOT WIN32)
    #find_package(Boost REQUIRED)
    #find_package(PCL REQUIRED COMPONENTS common)
    
    #add_subdirectory(src/api/HesaiLidar)
    #add_subdirectory(src/api/Driver_RpLidar_A1_M8)
    
    #set(SRC_FILES
    #    src/happly.h
    #    src/api/HesaiLidar/CLidarHesaiInterface.cpp
    #    src/api/HesaiLidar/CLidarHesaiInterface.h
    #    src/api/SlamwareLidar/CLidarSlamwareInterface.cpp
    #    src/api/SlamwareLidar/CLidarSlamwareInterface.h)
endif ()

add_library(${FILTER_NAME} SHARED
    ${SRC_FILES} 
    src/happly.h
    src/CLidarFilter.h
    src/CLidarFilter.cpp)

target_include_directories(${FILTER_NAME} 
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/api/Driver_RpLidar_A1_M8
        ${CMAKE_CURRENT_SOURCE_DIR}/src/api/HesaiLidar
        ${CMAKE_CURRENT_SOURCE_DIR}/src/api/HesaiLidar/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/api/HesaiLidar/include/PandarGeneralRaw/include
        ${CMAKE_SOURCE_DIR}/src/cyc_robots/Livox/src
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

target_link_directories(${FILTER_NAME}
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})
        
target_link_libraries(${FILTER_NAME}
    PUBLIC
        CycCore
        CycToolkit
        lslidar
        Robot_Livox
        fmt
        ${CyC_OPENCV_WORLD})

if (WIN32)
    target_link_libraries(${FILTER_NAME} PUBLIC libconfig++)
else ()
    target_link_libraries(${FILTER_NAME} PUBLIC config++)
endif (WIN32)

if (NOT WIN32)
    # Hesai Pandar library
    #target_link_libraries(${FILTER_NAME}
    #    PUBLIC
    #        ${Boost_LIBRARIES}
    #        ${PCL_IO_LIBRARIES}
    #        PandarGeneralSDK
    #        pcap)
endif ()

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

# Test unit
add_executable(tu_Lidar src/tu_Lidar.cpp)
target_include_directories(tu_Lidar PUBLIC ${CMAKE_SOURCE_DIR}/src/cyc_robots/Livox/src)
target_link_directories(tu_Lidar PUBLIC ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})
target_link_libraries(tu_Lidar PUBLIC CycToolkit Robot_Livox ${CyC_OPENCV_WORLD})

