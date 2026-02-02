cmake_minimum_required(VERSION 3.25)
project(PerchDetector VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(LibCamera REQUIRED HINTS cmake)
find_package(Gstreamer REQUIRED HINTS cmake)

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")

# # Utility for listing detected cameras
# add_executable(camscan camscan.cpp)
# target_link_libraries(camscan PRIVATE LibCamera ${OpenCV_LIBS})
# target_link_options(camscan PRIVATE -Wl,--copy-dt-needed-entries)


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
)
target_link_libraries(tasks PRIVATE util LibCamera ${OpenCV_LIBS})
target_include_directories(tasks PUBLIC tasks/include)

# # Library of helpers 
# add_library(cam_helpers STATIC
#     src/CameraWrapper.cpp 
#     src/ImageSender.cpp 
#     src/Depth.cpp
# )
# target_include_directories(cam_helpers PUBLIC include)
# target_link_libraries(cam_helpers PUBLIC LibCamera ${OpenCV_LIBS})
