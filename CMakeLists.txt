cmake_minimum_required(VERSION 3.5)
project(wego)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(wego opencv.cpp)
target_link_libraries(wego ${OpenCV_LIBS})