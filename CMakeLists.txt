cmake_minimum_required(VERSION 3.25)
project(PerchDetector VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(LibCamera REQUIRED HINTS cmake)
find_package(Gstreamer REQUIRED HINTS cmake)

# Utility for listing detected cameras
add_executable(camscan camscan.cpp)
target_link_libraries(camscan PRIVATE LibCamera ${OpenCV_LIBS})

# Library of helpers 
add_library(cam_helpers STATIC)
target_include_directories(cam_helpers PUBLIC include)
target_link_libraries(cam_helpers PUBLIC LibCamera ${OpenCV_LIBS})
target_sources(cam_helpers PRIVATE src/CameraWrapper.cpp src/ImageSender.cpp)

# Test for the helper
add_executable(camwrapper camwrapper.cpp)
target_include_directories(camwrapper PUBLIC include)
target_link_libraries(camwrapper PRIVATE cam_helpers)
target_link_options(camwrapper PRIVATE -Wl,--copy-dt-needed-entries)