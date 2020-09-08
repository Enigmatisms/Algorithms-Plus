#include "KF/Direct.hpp"
// #define INPUT_DEBUG

int main(int, char **){
    Eigen::Matrix3d K;
    K << 3968.297, 0, 1188.925,
        0, 3968.297, 979.657,
        0, 0, 1;
    cv::Mat p1 = cv::imread("/home/sentinel/testPics/data/im0.png");
    cv::Mat p2 = cv::imread("/home/sentinel/testPics/data/im1.png");
    std::cout << "P1, P2 shapes: " << p1.rows << ", " << p1.cols << ", " << p2.rows << ", " << p2.cols << std::endl;
    cv::Mat gray1, gray2;
    cv::cvtColor(p1, gray1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(p2, gray2, cv::COLOR_BGR2GRAY);
    ceres::Grid2D<uchar, 1> img(gray2.data, 0, gray2.rows, 0, gray2.cols);
    ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 1> > interpolator(img);
    Direct solver(K);

    std::cout << "K:\n";
    printMat<Eigen::Matrix3d>(K, 3);
    std::cout << "Kinv:\n";
    printMat<Eigen::Matrix3d>(K.inverse(), 3);

    int rows = p1.rows;
    int cols = p1.cols;

    float *data = new float [rows * cols];
    std::ifstream infile("/home/sentinel/testPics/data/data.bin", std::ios::in | std::ios::binary);
    std::cout << "Start to read.\n";
    infile.read((char *)data, sizeof(float) * cols * rows);
    std::cout << "End reading.\n";

    // std::cout << "Sample result: " << data[0] << ", " << data[100] << std::endl;

    solver.inputImages(gray1, gray2);       // 输入灰度图
    std::cout << "Image input completed.\n";

    #ifndef INPUT_DEBUG
        solver.optimization(interpolator, data, 3);
        std::cout << "Solver halts.\n";
        solver.drawResult(p1, data);            // 在彩色图上绘制结果
    #else
        solver.debugDraw(p1);
    #endif  //INPUT_DEBUG
    cv::resize(p1, p1, cv::Size(1480, 962));
    cv::imshow("disp", p1);
    cv::waitKey(0);
    
    delete [] data;
    std::cout << "Memory freed. Exiting...\n";
    return 0;
}