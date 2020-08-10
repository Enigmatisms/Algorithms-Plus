#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

cv::VideoCapture cap;
cv::Mat usb_frame;

int main(int, char **){
    cap.open(1);
    cap.read(usb_frame);
    printf("USB Cam starts to operate.\n");
    while(usb_frame.empty() != true){
        cap.read(usb_frame);
        if(!usb_frame.empty()){
            cv::imshow("usb", usb_frame);
            printf("USB image size(cols, rows):(%d, %d)\n", usb_frame.cols, usb_frame.rows);
        }
        cv::waitKey(5);
    }
    return 0;
}

    