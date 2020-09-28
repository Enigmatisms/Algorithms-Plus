#include "KF/Direct.hpp"
// #define INPUT_DEBUG

int main(int argc, char ** argv){
    Eigen::Vector4d K(718.856, 607.1928, 718.856, 185.2157);       // fx cx fy cy
    cv::Mat p1 = cv::imread("/home/sentinel/testPics/data3/left.png");
    char path[48];
    snprintf(path, 48, "/home/sentinel/testPics/data3/0%d.png", atoi(argv[1]));
    cv::Mat p2 = cv::imread(path);
    std::cout << "P1, P2 shapes: " << p1.rows << ", " << p1.cols << ", " << p2.rows << ", " << p2.cols << std::endl;
    cv::Mat gray1, gray2;
    cv::cvtColor(p1, gray1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(p2, gray2, cv::COLOR_BGR2GRAY);
    
    int rows = p1.rows;
    int cols = p1.cols;
    int pyr_num = 3;
    Direct solver(K, pyr_num);

    cv::Mat disparity = cv::imread("/home/sentinel/testPics/data3/disparity.png");
    float *data = new float [rows * cols], fx = 718.856, baseline = 0.573;
    uchar *dis_data = disparity.data;

    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            data[i * cols + j] = fx * baseline / (float)(dis_data[i * cols + j]);
        }
    }

    cv::Mat depth_map(disparity.rows, disparity.cols, CV_32FC1, data);
    std::vector<cv::Mat> depths(pyr_num);
    for (int i = 0; i < pyr_num; i++){
        double scale = pow(2, pyr_num - 1 - i);
        cv::resize(depth_map, depths[i], cv::Size(depth_map.cols / scale, depth_map.rows / scale));
    }

    solver.inputImages(gray1, gray2);       // 输入灰度图

    #ifndef INPUT_DEBUG
        solver.optimization(depths, 3);
        std::cout << "Solver halts.\n";
        solver.drawResult(p1, data);            // 在彩色图上绘制结果
    #else
        solver.debugDraw(p1);
    #endif  //INPUT_DEBUG
    cv::imshow("disp", p1);
    cv::waitKey(0);
    
    delete [] data;
    std::cout << "Memory freed. Exiting...\n";
    return 0;
}