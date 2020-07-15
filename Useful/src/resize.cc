#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int, char **){
    char path[40];
    for(int i = 1; i < 114; ++i){
        snprintf(path, 40, "/home/sentinel/pics/cam1/c1_%d.png", i);
        cv::Mat src = cv::imread(path);
        if(src.empty()) continue;
        cv::Mat dst(640, 480, CV_8UC3);
        cv::resize(src, dst, cv::Size(640, 480));
        snprintf(path, 40, "/home/sentinel/pics/cam3/c3_%d.png", i);
        cv::imwrite(path, dst);
        printf("Image(%d) has been processed.\n", i);
    }
    
    printf("Process completed.\n");
    return 0;
}