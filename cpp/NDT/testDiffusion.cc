#include <unistd.h>
#include "include/NDT.hpp"

int main(int argc, char* argv[]) {
    const std::string prefix = "/home/sentinel/Dataset/light/";
    cv::Mat origin;
    cv::Mat result;
    double sig_c = 1.0;
    int number = 0;
    if (argc >= 3) {
        sig_c = atoi(argv[2]);
    }
    if (argc >= 2) {
        number = atoi(argv[1]);
    }
    NDT ndt;
    if (number == -1) {
        double top[2] = {15, 3};
        double center[2] = {25, 40};
        double params[2] = {1.0, 4.0};
        origin.create(cv::Size(100, 100), CV_8UC1);
        ndt.drawDiffusion<double>(origin, top, center, params);
        cv::imshow("disp", origin);
        cv::waitKey(0);
    }
    else if (number == 0) {
        for (size_t i = 1; i < 400; i++) {
            std::string dir = prefix + std::to_string(i) + ".png";
            if (access(dir.c_str(), F_OK) == 0) {
                ndt.lightDiffusion(origin, result, i);
                printf("Image %lu processed.\n", i);
            }
        }
    }
    else {
        ndt.lightDiffusion(origin, result, number);
        cv::Mat src;
        cv::resize(origin, origin, cv::Size(4 * origin.cols, 4 * origin.rows));
        cv::resize(result, result, cv::Size(4 * result.cols, 4 * result.rows));
        cv::hconcat(origin, result, src);
        std::cout << "Number:" << number << ", sigma coeff: " << sig_c << ", argc: " << argc << std::endl;
        cv::imshow("disp", src);
        cv::waitKey(0);
    }
    cv::destroyAllWindows();
    return 0;
}
