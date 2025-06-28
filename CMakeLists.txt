cmake_minimum_required(VERSION 3.25)
project(PerchDetector VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(LibCamera REQUIRED HINTS cmake)
find_package(Gstreamer REQUIRED HINTS cmake)

add_executable(main src/main.cpp)
target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(main PRIVATE LibCamera ${OpenCV_LIBS})
