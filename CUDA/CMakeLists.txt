cmake_minimum_required(VERSION 2.8.3)
project(Task)
find_package( CUDA REQUIRED )

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

cuda_add_executable(
    Task
    testCuda.cu
)