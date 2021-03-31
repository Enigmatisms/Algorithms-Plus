#include "include/Predict.hpp"

int main() {
    double z = 8000;
    bool slow_judge = false;
    int delay_time = 10;
    Msg msg(0, 0, 15);
    Predict pre;

    cv::Mat img(1080, 1440, CV_8UC3);
    while (true) {
        cv::rectangle(img, cv::Rect(0, 0, 1440, 1080), cv::Scalar(0, 0, 0), -1);
        Eigen::Vector3d sim = pre.simulateTarget(z);
        Eigen::Vector3d pred;
        cv::Point3d cam_p(sim(0), sim(1), sim(2));
        pre.translatePredict(cam_p, msg, pred);
        pre.drawProjected(img, sim, pred);
        cv::imshow("pred", img);
        delay_time = slow_judge ? 200 : 10;
        char key = cv::waitKey(delay_time);
        if (key == 27) {
            break;
        }
        else if (key == 'e') {
            slow_judge = !slow_judge;
        }
        else if (key == ' ') {
            cv::waitKey(0);
        }
    }
    cv::destroyAllWindows();
    return 0;
}