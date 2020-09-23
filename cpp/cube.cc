#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::VideoCapture cap;
int grad_thresh = 80, wait_time = 1, yel_thresh = 19;

void morphologyProcess(const cv::Mat& src, cv::Mat& dst){
    cv::Mat gray, bin, res, flat;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::bitwise_not(gray, bin);
    
    cv::threshold(bin, bin, 215, 255, cv::THRESH_BINARY);                   // 对于低于阈值的部分保留，并且不改变值
    cv::dilate(bin, bin, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));

    cv::morphologyEx(gray, flat, cv::MORPH_GRADIENT, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(12, 12)));
    cv::dilate(flat, flat, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
    cv::bitwise_not(flat, flat);
    // 梯度外围消除
    cv::morphologyEx(gray, gray, cv::MORPH_GRADIENT, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8)));
    cv::threshold(gray, gray, grad_thresh, 255, cv::THRESH_BINARY);
    dst = bin - flat;       // - gray;
    cv::imshow("grad", gray);
    cv::imshow("binary", bin);
    cv::imshow("flat", flat);
}

void colorFiltering(const cv::Mat& src, cv::Mat& dst, int th, int dsz){
    cv::Mat channel[3];
    // cv::pyrMeanShiftFiltering(src, shift, 10, 10, 1, cv::TermCriteria(cv::TermCriteria::MAX_ITER, 1, 1));
    cv::split(src, channel);
    for (int i = 0; i < 3; i++){
        channel[i].convertTo(channel[i], CV_32FC1);
    }
    cv::Mat mean_rg = (channel[2] + channel[1]) / 2;
    cv::Mat yellow = mean_rg - channel[0];      // 红绿均值减去蓝色通道
    cv::Mat rg_diff = channel[2] - channel[1];  // 红绿通道差
    yellow.convertTo(yellow, CV_8UC1);
    rg_diff.convertTo(rg_diff, CV_8UC1);
    cv::threshold(yellow, yellow, th, 255, cv::THRESH_BINARY);
    cv::threshold(rg_diff, rg_diff, 9, 255, cv::THRESH_BINARY);
    dst = yellow - rg_diff;         // blue通道与rg均值差大于20,并且r - g不能大于9(否则过红，可能是皮肤)
    cv::erode(dst, dst, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8)));
    cv::dilate(dst, dst, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dsz, dsz)));
    cv::bitwise_not(dst, dst);
}

void frameCallBack(int pos, void *){
    cap.set(cv::CAP_PROP_POS_FRAMES, pos);
}

void gradThreshCallBack(int th, void*){
    grad_thresh = th;
}

void yellowThreshCallBack(int th, void*){
    yel_thresh = th;
}

int main(){
    char key;
    cv::Mat frame, res, _filter, inter;
    cv::Mat channels[3];
    cap.open("/home/sentinel/rav/task4.mp4");
    int frame_cnt = cap.get(cv::CAP_PROP_FRAME_COUNT), frame_pos = 0;
    cv::namedWindow("bilateral", cv::WINDOW_NORMAL);        // 双边滤波结果
    cv::namedWindow("grad", cv::WINDOW_NORMAL);             // 形态学梯度结果
    cv::namedWindow("binary", cv::WINDOW_NORMAL);           // 阈值化结果
    cv::namedWindow("filter", cv::WINDOW_NORMAL);           // 阈值化结果
    cv::namedWindow("result", cv::WINDOW_NORMAL);           // 最终结果
    cv::namedWindow("flat", cv::WINDOW_NORMAL);
    cv::createTrackbar("frames", "bilateral", &frame_pos, frame_cnt, frameCallBack);
    cv::createTrackbar("threshold", "grad", &grad_thresh, 255, gradThreshCallBack);
    cv::createTrackbar("yellow filtering", "filter", &yel_thresh, 25, yellowThreshCallBack);
    do{
        cap.read(frame);
        if (frame.empty() == true){
            break;
        }
        cv::bilateralFilter(frame, inter, 5, 128.0, 64.0);                        // 双边滤波
        colorFiltering(inter, _filter, yel_thresh, 40);
        morphologyProcess(inter, res);
        res -= _filter;
        cv::imshow("result", res);
        cv::imshow("filter", _filter);
        cv::imshow("bilateral", inter);
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