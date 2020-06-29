/**====================交替曝光影像录制节点========================
*/

#include "../include/CameraCtl.hpp"

cv::Mat frame;

char key = 0;


//摄像头图像发送节点
int main(int argc, char* argv[])
{
    //打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
    std::string outputVideoPath = "/home/sentinel/cv_output.avi";
    cv::Size sWH = cv::Size(1440, 1080);
	cv::VideoWriter outputVideo;
	outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 60.0, sWH);		
	bool record_judge = false, low = false;
    cm::CameraCtl ctl;
    ctl.startGrabbing();
    while (1) {
        if(low) ctl.setExposureTime(140);
        else ctl.setExposureTime(7000);
        low = !low;
        cv::Mat frame = ctl.getOpencvMat();				//录制视频
        if(frame.empty()) break;
	    if(record_judge) outputVideo<<frame;
        key = cv::waitKey(1);
		if(key=='e'){														//按下e键录像
			record_judge =! record_judge;
		}
        else if(key == 27) break;
		if(record_judge)
		    cv::circle(frame, cv::Point(15, 15), 8, cv::Scalar(0, 0, 255), -1);	//录像标识(左上角小圆点)
	    cv::imshow("disp", frame);
        
    }
    outputVideo.release();

	cv::destroyAllWindows();
	return 0;
}
