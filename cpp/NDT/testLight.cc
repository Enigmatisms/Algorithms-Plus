#include <unistd.h>
#include "include/NDT.hpp"

int main(int argc, char* argv[]) {
    const std::string prefix = "/home/sentinel/Dataset/light/";
    cv::Mat origin;
    cv::Mat result;
    double sig_c = 1.0;
    int number = 0;
    if (argc == 3) {
        sig_c = atof(argv[2]);
    }
    if (argc >= 2) {
        number = atoi(argv[1]);
    }
    NDT ndt;
    if (number == 0) {
        for (size_t i = 1; i < 400; i++) {
            std::string dir = prefix + std::to_string(i) + ".png";
            if (access(dir.c_str(), F_OK) == 0) {
                ndt.lightMatching(origin, result, i, sig_c);
                printf("Image %lu processed.\n", i);
            }
        }
    }
    else {
        cv::namedWindow("origin", cv::WINDOW_NORMAL);
        cv::namedWindow("result", cv::WINDOW_NORMAL);
        ndt.lightMatching(origin, result, number, sig_c);
        std::cout << "Number:" << number << ", sigma coeff: " << sig_c << ", argc: " << argc << std::endl;
        cv::imshow("origin", origin);
        cv::imshow("result", result);
        cv::waitKey(0);
    }
    return 0;
}
