cmake_minimum_required(VERSION 2.8.3)
project(Task CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
include_directories(./
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(Task1 rng.cc)
target_link_libraries(Task1
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(Task2 ceres_learn.cc)
target_link_libraries(Task2
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(Task3 ceres_test.cpp)
target_link_libraries(Task3
    ${CERES_LIBRARIES}
)