#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cmath>

// OpenCV 双边滤波的实现（只设定卷积核函数）
// 个人认为大概就是这样，每个像素点使用的核函数都是不同的（窗口一直在变）
// 也就是说，每一个像素都需要重新估计一个窗口，那么卷积操作实际上可以化为
// 对每一个像素，用周围点对对其进行加权平均，而不能像固定核函数一样，可以使用滑动卷积的形式
// 就是这样实现的

// 空间差异
float guass(int x0, int y0, int x1, int y1, float sig = 2){
    return exp(-(pow(x0 - x1, 2) + pow(y0 - y1, 2)) / 2 / pow(sig, 2));
}

// 像素光度差异
float guass(uchar p1, uchar p2, float sig = 2){
    return exp( - pow((float)p1 - (float)p2, 2) / 2 / pow(sig, 2));
}


uchar filter(uchar *data, int x, int y, int col, int size){
    float norm = 0.0, pix = 0.0;
    for (int i = x - size; i <= x + size; i++){
        for (int j = y - size; j < y + size; j++){
            float temp = guass(x, y, i, j, 2) * guass(data[x * col + y], data[i * col + j], 25);
            norm += temp;
            pix += (float)data[i * col + j] * temp;
        }
    }
    return (uchar)(pix / norm);
}

void bilateralFiltering(const cv::Mat& src, cv::Mat &dst, int size){
    dst = cv::Mat(src.rows, src.cols, src.type());
    uchar *data = src.data;
    uchar *dst_data = dst.data;
    int col = src.cols;
    for (int i = size; i < src.rows - size; i++){
        for (int j = size; j < src.cols - size; j++){
            dst_data[(i - size) * col + j - size] = filter(data, i, j, col, size);
        }
    }
}


int main(){
    cv::RNG rng;
    int padding_size = 4;
    cv::Mat img, _img = cv::imread("/home/sentinel/disp.jpg"), copy;
    cv::cvtColor(_img, img, cv::COLOR_BGR2GRAY);
    img.copyTo(copy);
    cv::Size origin(img.rows, img.cols);

    cv::copyMakeBorder(img, img, padding_size, padding_size, padding_size, padding_size, CV_HAL_BORDER_REPLICATE);
    // 对此图片进行双边滤波卷积
    cv::Mat dst(origin, CV_8UC1), sta;
    bilateralFiltering(img, dst, padding_size);

    cv::bilateralFilter(copy, sta, 9, 25, 25);
    std::cout << "RNG:" << std::endl;
    for (int i = 0; i < 10; i++){
        std::cout << rng.gaussian(1) << std::endl;
    }
    cv::imshow("disp", img);
    cv::imshow("bilateral", dst);
    cv::imshow("standard", sta);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}