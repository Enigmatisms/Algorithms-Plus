#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>

/// Integral image implementation
/// Add two rows (InPlace)
/// @param r the row to as addend, r+1 as summand
__global__ void integralByRow(float *arr, int r, int rows, int cols){
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= r * cols && index < (r + 1) * cols){
        arr[index + cols] += arr[index];     // Add 'row'th row to 'row + 1'th row
    }
}

// Add two cols (InPlace)
__global__ void integralByCol(float *arr, int c, int rows, int cols){
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index % rows == c){
        arr[index + 1] += arr[index]
    }
}

void integral(float *arr, int rows, int cols){
    for (int i = 0; i < rows - 1; i++){
        integralByRow(arr, i, rows, cols);
        cudaDeviceSynchronize();
    }
    for (int j = 0; j < cols - 1; j++){
        integralByCol(arr, j, rows, cols);
        cudaDeviceSynchronize();
    }
}

int main(){
    
    return 0;
}