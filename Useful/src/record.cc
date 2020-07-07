/**====================交替曝光影像录制节点========================
*/

#include "../include/CameraCtl.hpp"

cv::Mat frame, frame2;

char key = 0;


//摄像头图像发送节点
int main(int argc, char* argv[])
{
    //打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
    std::string outputVideoPath = "/home/sentinel/cv_output.avi";
    std::string outputVideoPath2 = "/home/sentinel/cv_output1.avi";
    cv::Size sWH = cv::Size(1440, 1080);
    cv::Size sWH2 = cv::Size(640, 480);
    cv::VideoCapture cap(0);
    cap.open(1);
	cv::VideoWriter outputVideo;
    cv::VideoWriter out2;
	outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 60.0, sWH);		
    out2.open(outputVideoPath2, CV_FOURCC('D', 'I', 'V', 'X'), 60.0, sWH2);
	bool record_judge = false, low = true;//false;
    cm::CameraCtl ctl;
    ctl.startGrabbing();
    while (1) {
        if(low) ctl.setExposureTime(140);
        else ctl.setExposureTime(7000);
        //low = !low;
        cv::Mat frame = ctl.getOpencvMat();				//录制视频
        cap.read(frame2);

        if(frame.empty()) break;
	    if(record_judge){
           outputVideo << frame;
           out2 << frame2;
        }
        key = cv::waitKey(1);
		if(key=='e'){														//按下e键录像
			record_judge =! record_judge;
		}
        else if(key == 27) break;
		if(record_judge)
		    cv::circle(frame, cv::Point(15, 15), 8, cv::Scalar(0, 0, 255), -1);	//录像标识(左上角小圆点)
	    cv::imshow("disp", frame);
        cv::imshow("cam2", frame2);
    }
    outputVideo.release();
    out2.release();

	cv::destroyAllWindows();
	return 0;
}
