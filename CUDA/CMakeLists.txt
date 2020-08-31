cmake_minimum_required(VERSION 2.8.3)
project(Task)
set(CMAKE_BUILD_TYPE "release")

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message("No appropriate opencv package found.")
endif()

set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 --use_fast_math
    -gencode=arch=compute_30,code=sm_30
    -gencode=arch=compute_35,code=sm_35
    -gencode=arch=compute_50,code=sm_50
    -gencode=arch=compute_52,code=sm_52
    -gencode=arch=compute_61,code=sm_61
    -gencode=arch=compute_70,code=sm_70
)

include_directories(
    ./include
    /usr/local/include
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    /usr/local/lib
)

cuda_add_executable(Task
    Integral.cu
)

target_link_libraries(Task
    ${OpenCV_LIBS}
)