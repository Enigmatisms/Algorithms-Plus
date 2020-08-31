#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <iostream>

// 下一步的学习计划：
// 1. cudaMallocPitch的联系（掌握2D数组操作）
// 2. 此后使用cuda结合opencv进行一个灰度图的显卡内阈值化操作

__global__ void get(int **arr, size_t x, size_t y){
    int ix = blockIdx.x;
    int iy = blockIdx.y;
    if (ix < x && iy < y){
        arr[ix][iy] = ix * iy;
    }
}

int main(){
    size_t x = 4, y = 2;
    int** arr = new int *[x];
    for (size_t i = 0; i < x; i++){
        arr[i] = new int [y];
    }
    std::cout << "Here1\n";

    // 如下的 cudaMalloc 将会出现问题， cudaMalloc在这里的行为和malloc不一样
    // int** arr_dev;
    // cudaMalloc((void **) &arr_dev, x * sizeof(int *));
    // std::cout << "Here1.5\n";
    // for (size_t i = 0; i < x; i++){
    //     cudaMalloc((void **) &arr_dev[i], y * sizeof(int));
    // }
    // cudaMallocPitch的设备指针仍然是一维的吗
    std::cout << "Here2\n";

    get<<<4, 2>>>(arr, x, y);
    std::cout << "Here3\n";

    cudaMemcpy(arr, arr_dev, x, cudaMemcpyDeviceToHost);
    for (size_t i = 0; i < y; i++){
        cudaMemcpy(arr[i], arr_dev[i], y, cudaMemcpyDeviceToHost);
    }
    std::cout << "Here4\n";


    for (size_t i = 0; i < x; i++){
        for (size_t j = 0; j < y; j++){
            printf("%d, ", arr_dev[i][j]);
        }
        printf("\n");
    }

    for (size_t i = 0; i < x; i++){
        cudaFree(arr_dev[i]);
        delete [] arr[i];
    }

    delete [] arr;
    cudaFree(arr_dev);
    printf("Memory freed.\n");
    return 0;
}