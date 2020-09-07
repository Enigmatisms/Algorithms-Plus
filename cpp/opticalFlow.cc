#include "KF/InverseComp.hpp"

int main(){
    InverseAdd opt;
    cv::Mat _pic1 = cv::imread("/home/sentinel/testPics/b1.jpg");
    cv::Mat _pic2 = cv::imread("/home/sentinel/testPics/b2.jpg");

    cv::resize(_pic1, _pic1, cv::Size(1200, 900));
    cv::resize(_pic2, _pic2, cv::Size(1200, 900));

    cv::Mat pic1, pic2;
    cv::cvtColor(_pic1, pic1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(_pic2, pic2, cv::COLOR_BGR2GRAY);

    // opt.debugDisplay(pic1, pic2);

    opt.inputImage(pic1, pic2);
    opt.calcOpticalFlow();
    opt.drawResult(pic1);


    cv::imshow("disp", pic1);
    cv::waitKey(0);

    return 0;
}