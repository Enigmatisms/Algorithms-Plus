#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

int main(int, char **){
    cv::VideoCapture cap(0);
    cap.open(1);
    cv::Mat frame;
    cap.read(frame);
    bool start = false;
    int _cnt = 0, pic_cnt = 1;
    while(!frame.empty()){
        if(_cnt % 10 == 0){
            _cnt = 0;
            if(start){
                char str[37];
                snprintf(str, 37, "/home/sentinel/RAV/pics/pic%d.png", pic_cnt);
                cv::imwrite(str, frame);
                ++pic_cnt;
            }
        }
        ++_cnt;
        if(pic_cnt > 84) break;

        if(start){
            cv::circle(frame, cv::Point(30, 30), 10, cv::Scalar(0, 0, 255), -1);
        }
        cv::imshow("disp", frame);
        char key = cv::waitKey(1);
        if(key == ' '){
            cv::waitKey(0);
        }
        else if(key == 27){
            break;
        }
        else if(key == 'e'){
            start = !start;
            if(start){
                printf("Start to save pics.\n");
            }
            else{
                printf("Cease saving pics.\n");
            }
        }
        cap.read(frame);
        double cexp = cap.get(CV_CAP_PROP_AUTO_EXPOSURE);
        cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 1.0);

        printf("Cap exp: %lf\n", cexp);
    }

    cap.release();
    cv::destroyAllWindows();    
    return 0;
}