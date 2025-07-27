cmake_minimum_required(VERSION 3.25)
project(PerchDetector VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(LibCamera REQUIRED HINTS cmake)
find_package(Gstreamer REQUIRED HINTS cmake)

# Utility for listing detected cameras
add_executable(camscan camscan.cpp)
target_link_libraries(camscan PRIVATE LibCamera ${OpenCV_LIBS})
target_link_options(camscan PRIVATE -Wl,--copy-dt-needed-entries)

# Library of helpers 
add_library(cam_helpers STATIC)
target_include_directories(cam_helpers PUBLIC include)
target_link_libraries(cam_helpers PUBLIC LibCamera ${OpenCV_LIBS})
target_sources(cam_helpers PRIVATE src/CameraWrapper.cpp src/ImageSender.cpp)

# Main testing program
add_executable(main main.cpp gui.cpp headless.cpp)
target_include_directories(main PUBLIC include)
target_link_libraries(main PRIVATE cam_helpers)
target_link_options(main PRIVATE -Wl,--copy-dt-needed-entries)

# Test for the visualization stream
add_executable(testfeed testfeed.cpp)
target_include_directories(testfeed PUBLIC include)
target_link_libraries(testfeed PRIVATE cam_helpers)
target_link_options(testfeed PRIVATE -Wl,--copy-dt-needed-entries)
