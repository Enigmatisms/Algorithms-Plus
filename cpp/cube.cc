#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::VideoCapture cap;
int thresh = 80, wait_time = 1;


void frameCallBack(int pos, void *){
    cap.set(cv::CAP_PROP_POS_FRAMES, pos);
}

void threshCallBack(int th, void*){
    thresh = th;
}

int main(){
    char key;
    cv::Mat frame, inter, gray, bin, res, flat;
    cv::Mat channels[3];
    cap.open("/home/sentinel/rav/task4.mp4");
    int frame_cnt = cap.get(cv::CAP_PROP_FRAME_COUNT), frame_pos = 0;
    cv::namedWindow("bilateral", cv::WINDOW_NORMAL);        // 双边滤波结果
    cv::namedWindow("grad", cv::WINDOW_NORMAL);             // 形态学梯度结果
    cv::namedWindow("binary", cv::WINDOW_NORMAL);           // 阈值化结果
    cv::namedWindow("result", cv::WINDOW_NORMAL);           // 最终结果
    cv::createTrackbar("frames", "bilateral", &frame_pos, frame_cnt, frameCallBack);
    cv::createTrackbar("threshold", "grad", &thresh, 255, threshCallBack);
    do{
        cap.read(frame);
        if (frame.empty() == true){
            break;
        }
        cv::split(frame, channels);
        cv::bilateralFilter(frame, inter, 5, 128.0, 64.0);
        cv::cvtColor(inter, gray, cv::COLOR_BGR2GRAY);
        cv::bitwise_not(gray, bin);
        cv::threshold(bin, bin, 210, 255, cv::THRESH_BINARY);                   // 对于低于阈值的部分保留，并且不改变值
        // cv::distanceTransform(bin, bin, cv::DIST_L1, 3, CV_8UC1);
        // cv::dilate(bin, bin, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        cv::morphologyEx(gray, flat, cv::MORPH_GRADIENT, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(12, 12)));
        cv::dilate(flat, flat, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
        cv::threshold(flat, flat, 50, 255, cv::THRESH_BINARY_INV);                  // flat 为无纹理（梯度几乎不变位置）

        // 梯度外围消除
        cv::morphologyEx(gray, gray, cv::MORPH_GRADIENT, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8)));
        cv::threshold(gray, gray, thresh, 255, cv::THRESH_BINARY);

        res = bin - gray - flat;
        cv::imshow("bilateral", inter);
        cv::imshow("grad", gray);
        cv::imshow("binary", bin);
        cv::imshow("result", res);
        key = cv::waitKey(wait_time);
        if (key == ' '){
            cv::waitKey(0);
        }
        else if (key == 27){
            break;
        }
        else if (key == 'e'){
            wait_time = 601 - wait_time;
        }
        frame_pos++;
        if (frame_pos % 5 == 0){
            cv::setTrackbarPos("frames", "bilateral", frame_pos);
        }
    } while(frame.empty() == false);
    cap.release();
    return 0;
}