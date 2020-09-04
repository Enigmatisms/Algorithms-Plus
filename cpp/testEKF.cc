#include "KF/EKF.hpp"
#include "KF/Circle.hpp"
// EKF： 在控制以及观测都存在噪声时（观测使用真实位置 + 高斯随机数）
// 对一个圆周运动的球体进行追踪预测

int main(int, char **){
    EKF<double, 3> filter(1, 2);
    Circle ball;
    cv::RNG rng;
    const cv::Point offset(120, 120);

    cv::Mat frame(240, 240, CV_8UC3);
    cv::namedWindow("Disp", cv::WINDOW_NORMAL);
    double now_t = (double)cv::getTickCount() / cv::getTickFrequency(), dt = 0.0, old_t = now_t;
    double pre_x = 0.0, pre_y = 0.0;

    char key = 0;
    cv::waitKey(29);
    for (int i = 0; i < 1000; i++){
        std::cout << "===================================================\n"; 
        cv::rectangle(frame, cv::Rect(0, 0, frame.cols, frame.rows), cv::Scalar(100, 100, 100), -1);
        now_t = (double)cv::getTickCount() / cv::getTickFrequency();
        dt = now_t - old_t;
        std::cout << ">>> dt:" << dt << std::endl;
        old_t = now_t;
        ball.move(dt);
        ball.draw(frame, offset);

        filter.predict(dt);
        filter.update(ball.x + rng.gaussian(5), ball.y + rng.gaussian(5), dt);
        filter.predictMove(dt, pre_x, pre_y);

        double start_t = (double)cv::getTickCount() / cv::getTickFrequency();
        cv::circle(frame, cv::Point(pre_x, pre_y) + offset, 8, cv::Scalar(0, 255, 255), -1);
        double delta_t = (double)cv::getTickCount() / cv::getTickFrequency() - start_t;
        std::cout << "Before drawing: execution time: " << delta_t << std::endl;
        cv::imshow("Disp", frame);
        key = cv::waitKey(29);
        if (key == 27){
            break;
        }
    }
    cv::destroyAllWindows();
    return 0;
}