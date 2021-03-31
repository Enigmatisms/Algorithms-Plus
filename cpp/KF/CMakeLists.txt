cmake_minimum_required(VERSION 2.8.3)
project(Predict)

## Torch 需要至少C++14进行编译
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 3 not found.")
endif()

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
   message(FATAL_ERROR "Eigen3 not found.")
endif()

find_package(Pangolin REQUIRED)
if(NOT Pangolin_FOUND)
   message(FATAL_ERROR "Pangolin not found.")
endif()

find_package(Ceres REQUIRED)
if (NOT Ceres_FOUND)
    message("No appropriate ceres package found.")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

SET(CMAKE_BUILD_TYPE "Release")

link_directories(
  /opt/MVS/lib/64
  /opt/MVS/lib/32
  /usr/local/lib
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ./include/
)

aux_source_directory(src/ PRE_SRC)

add_executable(Task KFTest.cc
  ${PRE_SRC}
)

target_link_libraries(Task
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  pthread
)