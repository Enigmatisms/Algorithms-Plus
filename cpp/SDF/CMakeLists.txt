cmake_minimum_required(VERSION 2.8.3)
project(Task)
set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_PREFIX_PATH "/home/sentinel/opencv-4.5.1/")

find_package(OpenCV 4.5.1 REQUIRED)
if (NOT OpenCV_FOUND)
    message("No appropriate opencv package found.")
endif()

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
   message(FATAL_ERROR "Eigen3 not found.")
endif()

set(OpenCV_INCLUDE_DIRS "/home/sentinel/opencv-4.5.1/include/")
set(${OpenCV_LIBS} "/home/sentinel/opencv-4.5.1/lib/")

include_directories(
    .
    ./include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    /usr/local/include/
)

link_directories(
    /usr/local/lib
)

aux_source_directory(
    ./src SRC
)

add_executable(Task sdf_test.cc
    ${SRC}
)

target_link_libraries(Task
    ${OpenCV_LIBS}
)