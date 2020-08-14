cmake_minimum_required(VERSION 2.8)
project( GrabOpenCV )
add_compile_options(-std=c++11)
find_package( OpenCV REQUIRED )
set(CMAKE_BUILD_TYPE "Release")
include_directories(. ${OpenCV_INCLUDE_DIRS} /opt/MVS/include)
link_directories(
    /opt/MVS/lib/64
    /opt/MVS/lib/32
)
aux_source_directory(. DIR_SRCS)
add_executable(GrabOpenCV GrabOpenCV.cc ${DIR_SRCS})
target_link_libraries(GrabOpenCV
    MvCameraControl
    pthread
    ${OpenCV_LIBS}
)
