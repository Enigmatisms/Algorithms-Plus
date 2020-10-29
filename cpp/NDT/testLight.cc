#include "include/NDT.hpp"

int main(int argc, char* argv[]){
    cv::Mat origin;
    cv::Mat result;
    double sig_c = 0.08;
    int number = 0;
    if (argc == 3){
        sig_c = (double)atof(argv[2]);
    }
    if (argc >= 2){
        number = atoi(argv[1]);
    }
    cv::namedWindow("origin", cv::WINDOW_NORMAL);
    cv::namedWindow("result", cv::WINDOW_NORMAL);
    NDT ndt;
    ndt.lightMatching(origin, result, number, sig_c);
    std::cout << "Number:" << number << ", sigma coeff: " << sig_c << ", argc: " << argc << std::endl;
    cv::imshow("origin", origin);
    cv::imshow("result", result);
    cv::waitKey(0);
    return 0;
}
