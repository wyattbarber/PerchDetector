cmake_minimum_required(VERSION 3.0)
project(PerchDetector VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

find_package(Iconv REQUIRED) # OpenCV link dependency
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LibCamera REQUIRED HINTS cmake)
find_package(jsoncpp REQUIRED)
find_package(Open3D REQUIRED HINTS $ENV{HOME}/open3d_install)

set(CMAKE_CXX_FLAGS "-DBLOCK_ALLOC") # Use custom allocator in default build
set(CMAKE_CXX_FLAGS_DEBUG "-fopenmp -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-fopenmp -Wall -O3")

# Generic logging and task interface library
add_library(util STATIC
    util/src/Task.cpp
    util/src/TaskManager.cpp
    util/src/BlockAllocator.cpp
    util/src/json_loader.cpp
)
target_include_directories(util PUBLIC util/include)
target_link_libraries(util PUBLIC jsoncpp_lib ${OpenCV_LIBS})

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
target_link_libraries(stsw_img040 PUBLIC util)
target_include_directories(stsw_img040 PUBLIC STSW_IMG040/Platform STSW_IMG040/VL53L8CX_ULD_API/inc)

# 3-D Hough Transform helpers
add_library(hough3d STATIC
    # hough3d/hough3d_tform.cpp
    hough3d/hough.cpp
    # hough3d/hough3dlines.cpp
    hough3d/pointcloud.cpp
    hough3d/sphere.cpp
    hough3d/vector3d.cpp
)
target_include_directories(hough3d PUBLIC hough3d)
target_link_libraries(hough3d PUBLIC Eigen3::Eigen)
target_compile_definitions(hough3d PUBLIC "HOUGH_VECTORIZE_VOTE")

# Library of specific task types
add_library(tasks STATIC 
    tasks/src/CameraWrapper.cpp
    tasks/src/CameraSim.cpp
    tasks/src/Disparity.cpp
    tasks/src/VL53L8CX.cpp
    tasks/src/LineFinder.cpp
    tasks/src/GrasperCtl.cpp
)
target_link_libraries(tasks PUBLIC util stsw_img040 LibCamera ${OpenCV_LIBS} Eigen3::Eigen hough3d Open3D::Open3D)
target_include_directories(tasks PUBLIC tasks/include)

# Main executable
add_executable(main main.cpp main_cmd.cpp main_exe.cpp)
target_link_libraries(main PRIVATE util tasks ${OpenCV_LIBS} LibCamera)
target_link_options(main PRIVATE -Wl,--copy-dt-needed-entries)