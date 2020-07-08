/**====================交替曝光影像录制节点========================
*/

#include "../include/CameraCtl.hpp"
#define GET_PICS

cv::Mat frame, frame2;

char key = 0;


//摄像头图像发送节点
int main(int argc, char* argv[])
{
    cv::VideoCapture cap(0);
    //打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
    cap.open(1);
    #ifndef GET_PICS
        std::string outputVideoPath = "/home/sentinel/cv_output.avi";
        std::string outputVideoPath2 = "/home/sentinel/cv_output1.avi";
        cv::Size sWH = cv::Size(1440, 1080);
        cv::Size sWH2 = cv::Size(640, 480);
	    cv::VideoWriter outputVideo;
        cv::VideoWriter out2;
	    outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 60.0, sWH);		
        out2.open(outputVideoPath2, CV_FOURCC('D', 'I', 'V', 'X'), 60.0, sWH2);
    #else
        char path[40];          
        int pic_cnt = 1, loop = 0;
    #endif  //GET_PICS    
	bool record_judge = false, low = false; //false;
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
            #ifndef GET_PICS
                outputVideo << frame;
                out2 << frame2;
            #else
                if(loop % 10 == 0){
                    loop = 0;
                    snprintf(path, 40, "/home/sentinel/RAV/pics/c1_%d.png", pic_cnt);
                    cv::imwrite(path, frame);
                    snprintf(path, 40, "/home/sentinel/RAV/pics/c2_%d.png", pic_cnt);
                    cv::imwrite(path, frame2);
                    ++pic_cnt;
                }
                ++loop;
            #endif  //GET_PICS  
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
    #ifndef GET_PICS
        outputVideo.release();
        out2.release();
    #else

    #endif  //GET_PICS

	cv::destroyAllWindows();
	return 0;
}
