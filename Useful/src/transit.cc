#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>

int main(int argc, char * argv[]){
    std::string path = argv[1];
    std::string opath = argv[2];
    cv::Mat frame;
    cv::VideoCapture cap;
    cv::VideoWriter wri;
    int width = atoi(argv[3]),
        height = atoi(argv[4]);
    printf("argv[3]: %d, argv[4]: %d\n", width, height);
    cv::Size sWH = cv::Size(width, height);
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
