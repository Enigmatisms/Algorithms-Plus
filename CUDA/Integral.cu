#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/// 实现了cuda并行的灰度化以及阈值化

// cuda 与图像处理结合
__global__ void threshold(uchar *data, int cols, int thresh){
    int row = blockIdx.y;       // 行号
    int col = blockIdx.x;       // 列号
    if (data[row * cols + col] > thresh){
        data[row * cols + col] = 255;
    }
    else{
        data[row * cols + col] = 0;
    }
}

// 输入一个三通道图像的data (device)
// 输出一个灰度图的data (device)
__global__ void grayScale(uchar *src, uchar *dst, int cols){
    int row = blockIdx.y;
    int col = blockIdx.x;
    dst[row * cols + col] = 
        (uchar)(((float)src[(row * cols + col) * 3] + 
         (float)src[(row * cols + col) * 3 + 1] +
         (float)src[(row * cols + col) * 3 + 2]) / 3);
}

int main(){
    cv::Mat frame = cv::imread("/home/sentinel/test.jpg");
    int num = frame.rows * frame.cols;
    size_t size = num * sizeof(uchar);
    dim3 grid(frame.cols, frame.rows);

    uchar *frame_dev, *gray_dev, *data_dev, *gray_data = new uchar[num], *th_data = new uchar[num]; 
    cudaMalloc((void **) &frame_dev, 3 * size);
    cudaMalloc((void **) &data_dev, size);
    cudaMalloc((void **) &gray_dev, size);
    cudaMemcpy(frame_dev, frame.data, 3 * size, cudaMemcpyHostToDevice);
    grayScale<<<grid, 1>>>(frame_dev, gray_dev, frame.cols);
    cudaDeviceSynchronize();

    cudaMemcpy(data_dev, gray_dev, size, cudaMemcpyDeviceToDevice);
    threshold<<<grid, 1>>>(data_dev, frame.cols, 120);
    cudaDeviceSynchronize();

    // OPENCV mat的数据结构组织：x * cols + y x为行号 y 为列号
    cudaMemcpy(gray_data, gray_dev, size, cudaMemcpyDeviceToHost);
    cudaMemcpy(th_data, data_dev, size, cudaMemcpyDeviceToHost);

    cv::Mat gray(frame.rows, frame.cols, CV_8UC1, gray_data);
    cv::Mat th(frame.rows, frame.cols, CV_8UC1, th_data);
    cv::imshow("gray", gray);
    cv::imshow("threshold", th);

    cv::waitKey(0);
    
    delete [] gray_data;
    delete [] th_data;
    cudaFree(data_dev);
    cudaFree(gray_dev);
    cudaFree(frame_dev);
    printf("Data freed. Shutting down...\n");
    return 0;
}