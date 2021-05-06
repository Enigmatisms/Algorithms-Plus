#include "include/Predict.hpp"

double coeff_vx = 16;
double coeff_ax = 32; 
double speed_thresh = 8;
double acc_thresh = 5;

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: ./Task <use Tanh?> <coeff_vx> <coeff_ax>";
        return -1;
    }
    bool use_tanh = atoi(argv[1]);
    coeff_vx = atof(argv[2]);
    coeff_ax = atof(argv[3]);
    double z = 5000;
    bool slow_judge = false;
    int delay_time = 10;
    Msg msg(0, 0, 15);
    Predict pre;

    cv::Mat img(1080, 1440, CV_8UC3);
    while (true) {
        cv::rectangle(img, cv::Rect(0, 0, 1440, 1080), cv::Scalar(0, 0, 0), -1);
        Eigen::Vector3d sim = pre.simulateTarget(z, SimType(1 - use_tanh));
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