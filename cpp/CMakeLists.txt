cmake_minimum_required(VERSION 2.8.3)
project(Task)
set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message("No appropriate opencv package found.")
endif()

find_package(Eigen3 REQUIRED)
if (NOT Eigen3_FOUND)
    message("No appropriate libeigen3 package found.")
endif()

include_directories(
    .
    ./KF
    /usr/local/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(
    /usr/local/lib
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
add_executable(Task opticalFlow.cc
)

target_link_libraries(Task
    ${OpenCV_LIBS}
)