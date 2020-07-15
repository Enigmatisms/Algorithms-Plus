//======================视频逐帧调试========================

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "../include/FindLight.hpp"

char key = 0;
std::string inputVideoPath = "/home/sentinel/standard/cv_d85a30.avi";
cv::VideoCapture cap(inputVideoPath);
int nowPos = 0, frameNum = (int)cap.get(cv::CAP_PROP_FRAME_COUNT),
            key_wait = 1;

void slideCallBack(int pos, void *){
    cap.set(cv::CAP_PROP_POS_FRAMES, pos);
}


//摄像头图像发送节点
int main(int argc, char* argv[])
{
    FindLight fl;
    //打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
    printf("Frame num: %d\n", frameNum);
    cv::Mat img;
    cv::namedWindow("disp", cv::WINDOW_NORMAL);
    cv::namedWindow("trackers", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Pos", "trackers", &nowPos, frameNum, slideCallBack);

    printf("Cap opened? %d with %d frames.\n", cap.isOpened(), (int)cap.get(cv::CAP_PROP_FRAME_COUNT));
    while (1) {
        nowPos = (int)cap.get(cv::CAP_PROP_POS_FRAMES);
        if(nowPos % 10 == 0){
            cv::setTrackbarPos("Pos", "trackers", nowPos);
        }
        cap >> img;
        if(img.empty()) break;
        cv::imshow("disp", img);
        char key = cv::waitKey(key_wait);
        if(key == ' '){
            cv::waitKey(0);
        }
        else if(key == 'e'){
            key_wait = 600 - key_wait;
        }
        else if(key == 27){
            break;
        }
    }
    cap.release();
	cv::destroyAllWindows();
	return 0;
}
