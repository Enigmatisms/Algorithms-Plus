#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>

int main(int argc, char * argv[]){
    std::string path = argv[1];
    std::string opath = argv[2];
    cv::Mat frame;
    cv::VideoCapture cap;
    cv::VideoWriter wri;
    cv::Size sWH = cv::Size(1440, 1080);
	wri.open(opath, CV_FOURCC('D', 'I', 'V', 'X'), 60.0, sWH);		
    cap.open(path);
    cap.read(frame);
    while(frame.empty() == false){
        wri.write(frame);
        cap.read(frame);
    }
    printf("Transition completes.\n");
    wri.release();
    cap.release();
    return 0;
}