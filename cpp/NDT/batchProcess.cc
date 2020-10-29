#include "include/NDT.hpp"

int main(int argc, char* argv[]){
    cv::Mat origin;
    cv::Mat result;
    std::string path_prefix = "/home/sentinel/light/out/";
    double sig_c = 0.08;
    if (argc == 2){
        sig_c = (double)atof(argv[1]);
    }
    for (int i = 0; i < 406; i++){
        NDT ndt;
        ndt.lightMatching(origin, result, i, sig_c);
        cv::Mat output(cv::Size(origin.cols, origin.rows), CV_8UC3);
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
        cv::hconcat(origin, result, output);
        std::string path = path_prefix + std::to_string(i) + ".png";
        cv::imwrite(path, output);
        std::cout << "Image " + std::to_string(i) + ".png" + " completed.\n";
    }
    std::cout << "Process completed.\n";
    return 0;
}
