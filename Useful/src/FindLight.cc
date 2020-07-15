#include "../include/FindLight.hpp"

FindLight::FindLight(){
    ;
}

FindLight::~FindLight(){
    ;
}

void FindLight::preProcess(cv::Mat &src){
    cv::Mat proced;
    cv::Mat channels[3];
    cv::split(src, channels);
    cv::imshow("hsv", proced);
}