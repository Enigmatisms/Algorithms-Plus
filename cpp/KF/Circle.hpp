#ifndef __CIRCLE__
#define __CIRCLE__
#define PI 3.141592653589793

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Circle{
public:
    Circle(){
        x = 100.0;
        y = 0.0;
        w = PI / 180 / 0.03 * 10;       // 大约是每30毫秒10度
        now_t = 0;
    }
    ~Circle(){;}
public:
    void move(double dt);
    void draw(cv::Mat &src, const cv::Point& offset);
    double updateAngularVel(double dt){
        now_t += dt * 6;
        return 1.5 * sin(now_t) + 6.0;
    }
public:
    double x;
    double y;
    double w;
    double now_t;
    cv::RNG rng;
};

void Circle::move(double dt){
    double actual_w = updateAngularVel(dt) + rng.gaussian(1);       // 高斯噪声的速度
    double temp_x = x * cos(actual_w * dt) - y * sin(actual_w * dt);
    double temp_y = y * cos(actual_w * dt) + x * sin(actual_w * dt);
    x = temp_x;
    y = temp_y;
    std::cout << "Ground truth:" << x << ", " << y << ", " << actual_w << std::endl;
}

void Circle::draw(cv::Mat& src, const cv::Point& offset){
    cv::circle(src, cv::Point(x, y) + offset, 8, cv::Scalar(255, 0, 0), -1);
}


#endif // __CIRCLE__