# a cmake file at first try

cmake_minimum_required(VERSION 2.8)
project(eigenT)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE release)
ENDIF()

add_compile_options(-std=c++11)

MESSAGE("CMake build type: " ${CMAKE_BUILD_TYPE})

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
    /usr/local/lib
)

if(Eigen3_FOUND)
    message("Eigen3 installed in this project!")
endif()

if(OpenCV_FOUND)
    message("OpenCV required package is found!")
endif()

add_executable(
    eigenT
    eigenTest.cc
)
