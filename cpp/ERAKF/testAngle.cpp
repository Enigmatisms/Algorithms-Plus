#include <ctime>
#include "serial_com/include/serial_com/LOG.hpp"
#include <opencv2/core.hpp>
#include "Prediction.hpp"

#define RAD2DEG 57.2958
#define print rmlog::LOG::printc
//#define RECORD

double start_t = 0.0, end_t = 0.0, sum_t = 0.0;
double frame_cnt = 0;
bool can_input = true;
int mouse_x = 0, mouse_y = 0;

AngPre ap;

static void onMouse(int event, int x, int y, int flags, void *param){
    if(can_input){
        ap.queueInput(x, y);
        can_input = false;
        mouse_x = x;
        mouse_y = y;
    }
}

int main(){
    #ifdef RECORD
    std::string outputVideoPath = "cv_output.avi";
    cv::Size sWH = cv::Size(1000, 1000);
	cv::VideoWriter outputVideo;
	outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 25.0, sWH);
    #endif	

    /*float pitch = 0.0, yaw = 0.0, delay = 0.0;
    ballistic::GimbalCtrl g_ctrl;
    g_ctrl.Init(0, 7.95, -4.56, 0.0, 0.0, 16.0, 0.017772);
    for(int i = -500; i<= 500; i+= 100){
        for(int j = 500; j>=-500; j-=100){                 //由于y轴是在图像上向上为负
            g_ctrl.Transform(cv::Point3f(i, j, 2500), pitch, yaw, delay);
            std::cout<<"Result for(x, y, z):"<<(float)i/1000<<", "<<
                (float)j/1000<<", "<<5.0<<std::endl;
            std::cout<<"Pitch & Yaw:"<<pitch<<", "<<yaw<<std::endl;
            std::cout<<"Delay time:"<<delay<<std::endl<<std::endl;
        }
    }
    float pitch2 = 6.2345;
    unsigned int p = (*((unsigned int *)&pitch2));
    std::cout<<"P:"<<p<<std::endl;
    std::cout<<(unsigned int *)&pitch2<<std::endl;*/
    /*std::cout<<"Three times for error:"<<std::endl;
    for(int j = 500; j >=-500; j-=500){
        g_ctrl.Transform(cv::Point3f(0, j, 5000), pitch, yaw);
        std::cout<<std::endl;
    }*/
    cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
    cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::setMouseCallback("disp", onMouse, 0);

    int cnt = 0;
    while(1){
        cv::rectangle(img, cv::Point(0, 0), cv::Point(1000, 1000), cv::Scalar(100, 100, 100), -1);\
        cv::circle(img, cv::Point2i(mouse_x, mouse_y), 4, cv::Scalar(200, 0, 0), -1);
        if(ap.testPredict(img) == false){
            cv::imshow("disp", img);
            break;
        }
        cv::imshow("disp", img);
        char key = cv::waitKey(1);            //阻滞9ms
        if(key == 27) break;
        can_input = true;

        #ifdef RECORD
            outputVideo << img;
        #endif


        #ifdef MOUSE_RECORD
        ++cnt;
        if(cnt > 500){          // 帧数限制
            break;
        }
        #endif
    }

    #ifdef RECORD
        outputVideo.release();
    #endif
    //保存标准鼠标事件录制

    #ifdef MOUSE_RECORD     
        ap.rcd.writeStrInFile("/home/sentinel/self-targeting/py_standard.txt");
    #endif //MOUSE_RECORD
    
    //保存Kalman滤波事件
    #ifdef RECORD_TEST
        ap.rcd.writeStrInFile();   //默认路径"pyf.txt"
    #endif //RECORD_TEST

    printf("Output completed\n");
    cv::destroyAllWindows();
    return 0;
}
