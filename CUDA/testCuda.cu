#include <cstdio>
#include <cmath>
#include <iostream>
#include <driver_types.h>                   // 包含cuda的内置类型如cudaError_t
#include <cuda_runtime.h>                   // 包含诸多cuda函数
#include <device_launch_parameters.h>       // 包含 blockIdx等参数

static void CheckCudaErrorAux (const char *, unsigned, const char *, cudaError_t);
#define CUDA_CHECK_RETURN(value) CheckCudaErrorAux(__FILE__,__LINE__, #value, value)

/**
 * stixels 中别人已经写好的 返回值检查函数
 */
static void CheckCudaErrorAux (const char *file, unsigned line, const char *statement, cudaError_t err) {
	if (err == cudaSuccess)
		return;
	std::cerr << statement<<" returned " << cudaGetErrorString(err) << "("<<err<< ") at "<<file<<":"<<line <<
			std::endl;
	exit (1);
}


// self increment
__global__ void incrementArray(float *arr, int N){
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index < N){
        arr[index] = arr[index] + 1.0f;
    }
}

// 两个数组相加
__global__ void ArrayAdd(float *a, float *b, float *dst, int N){
    int index = blockIdx.x;
    if (index < N){
        dst[index] = a[index] + b[index];
    }
}

__global__ void ArrayInitialize(float *src, int N){
    int index = blockIdx.x;
    if (index < N){
        src[index] = (float)index;
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
    CUDA_CHECK_RETURN(cudaMalloc((void **) &a_dev, size));

    // cudaMemcpy(&dst, &src, size in bytes, way)
    CUDA_CHECK_RETURN(cudaMemcpy(a_dev, a_host, size, cudaMemcpyHostToDevice));

    int blockSize = 4;
    int nBlocks = N / blockSize + (N % blockSize == 0 ? 0 : 1);
    incrementArray <<<nBlocks, blockSize>>>(a_dev, N);

    CUDA_CHECK_RETURN(cudaMemcpy(b_host, a_dev, size, cudaMemcpyDeviceToHost));

    for (int i = 0; i < N - 1; i++){
        printf("%f, ", b_host[i]);
    }
    printf("%f\n", b_host[N - 1]);

    free(a_host);
    free(b_host);

    CUDA_CHECK_RETURN(cudaFree(a_dev));

    float *b_dev, *c_dev, c_host[N];
    CUDA_CHECK_RETURN(cudaMalloc((void **) &a_dev, N * sizeof(float)));
    CUDA_CHECK_RETURN(cudaMalloc((void **) &b_dev, N * sizeof(float)));
    CUDA_CHECK_RETURN(cudaMalloc((void **) &c_dev, N * sizeof(float)));

    ArrayInitialize<<<64, 1>>>(a_dev, N);   // 果然与block的设置数量有关
    ArrayInitialize<<<64, 1>>>(b_dev, N);
    ArrayInitialize<<<64, 1>>>(c_dev, N);

    ArrayAdd<<<64, 1>>>(a_dev, b_dev, c_dev, N);

    CUDA_CHECK_RETURN(cudaMemcpy(c_host, c_dev, N * sizeof(float), cudaMemcpyDeviceToHost));

    printf("Array adding result:\n");
    for (int i = 0; i < N - 1; i++){
        printf("%f, ", c_host[i]);
    }
    printf("%f\n", c_host[N - 1]);

    CUDA_CHECK_RETURN(cudaFree(a_dev));
    CUDA_CHECK_RETURN(cudaFree(b_dev));
    CUDA_CHECK_RETURN(cudaFree(c_dev));

    printf("CUDA device memory cleaned.\n");
    return 0;
}