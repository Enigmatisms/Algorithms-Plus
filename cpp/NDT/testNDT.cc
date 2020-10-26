#include "include/NDT.hpp"

int main(){
    cv::Mat origin(cv::Size(2 * X_SIG, 2 * Y_SIG), CV_8UC3);
    cv::Mat result(cv::Size(X_SIG * 2 + 4 * TRANS_X, 2 * Y_SIG + 4 * TRANS_Y), CV_8UC3);
    cv::Mat perturb(cv::Size(X_SIG * 2 + 4 * TRANS_X, 2 * Y_SIG + 4 * TRANS_Y), CV_8UC3);
    cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("origin", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("perturb", cv::WINDOW_AUTOSIZE);
    double angle;
    cv::Point2d trans;
    std::vector<cv::Point2d> vec;
    NDT ndt;
    std::cout << ">>> NDT problem initialized.\n";
    ndt.initDistribute(origin);
    std::cout << ">>> Initial distribution established.\n";
    ndt.getPerturbSim(perturb, vec, angle, trans);
    ndt.minimize(vec, result, angle);
    std::cout << ">>> Real rotation: " << angle << std::endl;
    std::cout << ">>> Translation ground truth: " << trans.x << ", " << trans.y << std::endl;
    std::cout << ">>> Solver completed.\n";
    cv::imshow("origin", origin);
    cv::imshow("result", result);
    cv::imshow("perturb", perturb);
    cv::waitKey(0);
    return 0;
}
