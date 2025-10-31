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

# Library of helpers 
add_library(cam_helpers STATIC)
target_include_directories(cam_helpers PUBLIC include)
target_link_libraries(cam_helpers PUBLIC LibCamera ${OpenCV_LIBS})
target_sources(cam_helpers PRIVATE src/CameraWrapper.cpp src/ImageSender.cpp src/Depth.cpp)

# # Test for the visualization stream
# add_executable(testfeed testfeed.cpp)
# target_include_directories(testfeed PUBLIC include)
# target_link_libraries(testfeed PRIVATE cam_helpers)
# target_link_options(testfeed PRIVATE -Wl,--copy-dt-needed-entries)

# # Calibration data collector
# add_executable(calibrate calibrate.cpp src/stereocam.cpp)
# target_include_directories(calibrate PUBLIC include)
# target_link_libraries(calibrate PRIVATE cam_helpers)
# target_link_options(calibrate PRIVATE -Wl,--copy-dt-needed-entries)

add_executable(capture capture.cpp src/stereocam.cpp)
target_include_directories(capture PUBLIC include)
target_link_libraries(capture PRIVATE cam_helpers)
target_link_options(capture PRIVATE -Wl,--copy-dt-needed-entries)

# # Rectified image grabber
# add_executable(capture_rectify capture_rectify.cpp src/stereocam.cpp)
# target_include_directories(capture_rectify PUBLIC include)
# target_link_libraries(capture_rectify PRIVATE cam_helpers)
# target_link_options(capture_rectify PRIVATE -Wl,--copy-dt-needed-entries)

# # Main testing program
# add_executable(main main.cpp src/gui.cpp src/headless.cpp src/stereocam.cpp src/visualization.cpp)
# target_include_directories(main PUBLIC include)
# target_link_libraries(main PRIVATE cam_helpers)
# target_link_options(main PRIVATE -Wl,--copy-dt-needed-entries)

