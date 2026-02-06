cmake_minimum_required(VERSION 3.25)
project(PerchDetector VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(LibCamera REQUIRED HINTS cmake)
find_package(Gstreamer REQUIRED HINTS cmake)

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")

# Generic logging task interface library
add_library(util STATIC
    util/src/Task.cpp
)
target_include_directories(util PUBLIC util/include)

# VL53L8CX API library
add_library(stsw_img040 STATIC 
    STSW_IMG040/Platform/platform.cpp
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_api.c
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_plugin_detection_thresholds.c
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_plugin_motion_indicator.c
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_plugin_xtalk.c
)
set_source_files_properties(
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_api.c
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_plugin_detection_thresholds.c
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_plugin_motion_indicator.c
    STSW_IMG040/VL53L8CX_ULD_API/src/vl53l8cx_plugin_xtalk.c

    PROPERTIES LANGUAGE CXX
)
target_link_libraries(stsw_img040 PRIVATE util)
target_include_directories(stsw_img040 PUBLIC STSW_IMG040/Platform STSW_IMG040/VL53L8CX_ULD_API/inc)

# Library of specific task types
add_library(tasks STATIC 
    tasks/src/CameraWrapper.cpp
    tasks/src/Depth.cpp
)
target_link_libraries(tasks PRIVATE util LibCamera ${OpenCV_LIBS})
target_include_directories(tasks PUBLIC tasks/include)

# Main executable
add_executable(main main.cpp)
target_link_libraries(main PRIVATE util tasks ${OpenCV_LIBS} LibCamera)
target_link_options(main PRIVATE -Wl,--copy-dt-needed-entries)