# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(Task1)
set(CMAKE_BUILD_TYPE "Release")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)  
include_directories(. ${OpenCV_INCLUDE_DIRS} /opt/MVS/include
    ./include
)
link_directories(
    /opt/MVS/lib/64
    /opt/MVS/lib/32
)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
aux_source_directory(src PROJ_SRC)
aux_source_directory(src/aim_deps PROJ_DEPS)
aux_source_directory(src/distance PROJ_DIST)

# Declare the executable target built from your sources
add_executable(Task1 src/info.cc
    ${PROJ_DEPS}
    ${PROJ_DIST}
)
target_link_libraries(Task1
    MvCameraControl
    ${OpenCV_LIBS}
    pthread
    libMvCameraControl.so
    /usr/lib/x86_64-linux-gnu/libtiff.so.5.2.4
)
