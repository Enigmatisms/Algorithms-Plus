#include <cstdio>
#include <cmath>
#include <cuda_runtime.h>


// self increment
__global__ void incrementArray(float *arr, int N){
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index < N){
        arr[index] = arr[index] + 1.0f;
    }
}

int main(){
    float* a_host, *a_dev;
    int N = 64;
    size_t size = N * sizeof(float);

    a_host = (float *)malloc(size);
    float *b_host = (float *)malloc(size);

    for (int i = 0; i < N; i++){
        a_host[i] = (float) i;
    }
    cudaMalloc((void **) &a_dev, size);

    // cudaMemcpy(&dst, &src, size in bytes, way)
    cudaMemcpy(a_dev, a_host, size, cudaMemcpyHostToDevice);

    int blockSize = 4;
    int nBlocks = N / blockSize + (N % blockSize == 0 ? 0 : 1);
    incrementArray <<<nBlocks, blockSize>>>(a_dev, N);

    cudaMemcpy(b_host, a_dev, size, cudaMemcpyHostToDevice);

    for (int i = 0; i < N; i++){
        printf(", %d" + !i, i);
    }
    printf("\n");

    free(a_host);
    free(b_host);

    cudaFree(a_dev);

    printf("CUDA device memory cleaned.\n");
    return 0;
}