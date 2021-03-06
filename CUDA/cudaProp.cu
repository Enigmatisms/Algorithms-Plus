#include <cuda_runtime.h>
#include <cstdio>
#include <iostream>

// 输出CUDA内置的相关信息
// Device name: GeForce MX150
// Kernel version: code_61
// Total Global Mem: 2099904512
// Shared Mem per block: 49152
// Warp size: 32
// Max thread per block: 1024
// Max thread dim: 1024, 1024, 64
// Max Grid size: 2147483647, 65535, 65535


int main(){
    cudaDeviceProp prop;
    int count = 0;
    cudaGetDeviceCount(&count);
    for (int i = 0; i < count; i++){
        cudaGetDeviceProperties(&prop, i);
    }
    std::cout << "Device name: " << prop.name << std::endl;
    std::cout << "Kernel version: code_" << prop.major  << prop.minor << std::endl;
    std::cout << "Total Global Mem: " << prop.totalGlobalMem << std::endl;
    std::cout << "Shared Mem per block: " << prop.sharedMemPerBlock << std::endl;
    std::cout << "Warp size: " << prop.warpSize << std::endl;
    std::cout << "Max thread per block: " << prop.maxThreadsPerBlock << std::endl;
    std::cout << "Max thread dim: " << prop.maxThreadsDim[0] << ", " << prop.maxThreadsDim[1] <<
             ", " << prop.maxThreadsDim[2] << std::endl;
    std::cout << "Max Grid size: " << prop.maxGridSize[0] << ", " << prop.maxGridSize[1] <<
             ", " << prop.maxGridSize[2] << std::endl;
    return 0;
}