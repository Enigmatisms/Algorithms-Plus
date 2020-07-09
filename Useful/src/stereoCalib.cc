#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>

const int x_num = 6;        // 横向有6个角点
const int y_num = 8;        // 纵向有8个角点
const int image_num = 113;
const int thread_num = 8;   // 使用线程数
const cv::Size _sz(x_num, y_num);

std::mutex mtx;

std::vector<std::vector<cv::Point2f> > corner_b;
std::vector<std::vector<cv::Point2f> > corner_t;

std::vector<std::vector<cv::Point2f> > pts_b;
std::vector<std::vector<cv::Point2f> > pts_t;

std::vector<std::vector<cv::Point3f> > world_pts;
cv::Mat R, T, E, F;


/// 计算世界坐标点
void worldPoints(std::vector<std::vector<cv::Point3f> > &wps, int len, int xnum, int ynum, int img_num){
    std::vector<cv::Point3f> pts;
    for(int i = 0; i < xnum; ++i){
        for(int j = 0; j < ynum; ++j){
            pts.emplace_back(cv::Point3f(len * i, len * j, 0));
        }
    }
    for(int i = 0; i < img_num; ++i){
        wps.emplace_back(pts);
    }
}

void printMat(const cv::Mat &src){
    if(src.empty()){
        printf("Mat is empty.\n");
    }
    else{
        for(int i = 0; i < src.rows; ++i){
            printf("[");
            for(int j = 0; j < src.cols - 1; ++j){
                printf("%lf, ", src.at<double>(i, j));
            }
            printf("%lf],\n", src.at<double>(i, src.cols - 1));
        }
    }
    printf("\n");
}

/// @brief 线程函数, 从第start张图 到 第ending张图 进行图像处理
void findCorners(int start, int ending){
    char path1[40];
    char path2[40];
    for(int i = start; i < ending; ++i){

        /// 下方摄像头
        printf("Camera image %d being processed.\n", i);
        snprintf(path1, 40, "/home/sentinel/pics/cam1/c1_%d.png", i);
        snprintf(path2, 40, "/home/sentinel/pics/cam2/c2_%d.png", i);
        cv::Mat src = cv::imread(path1);
        cv::Mat src2 = cv::imread(path2);
        if(src.empty() || src2.empty()) continue;
        std::vector<cv::Point2f> tmp_b;
        cv::Mat dst(src.rows, src.cols, CV_8UC1);
        cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
        cv::findChessboardCorners(dst, _sz, tmp_b);
        if(tmp_b.size() != 48){
            printf("Process skipped. (B)Image %d has less than 48 corner vertices.\n", i);
            pts_b.emplace_back(tmp_b);

        }
        else{
            cv::find4QuadCornerSubpix(dst, tmp_b, cv::Size(5, 5));      /// 亚像素化角点
            pts_b.emplace_back(tmp_b);
        }

        /// 上方摄像头
        std::vector<cv::Point2f> tmp_t;
        cv::Mat dst2(src.rows, src.cols, CV_8UC1);
        cv::cvtColor(src2, dst2, cv::COLOR_BGR2GRAY);
        cv::findChessboardCorners(dst2, _sz, tmp_t);
        if(tmp_t.size() != 48){
            printf("Process skipped. (T)Image %d has less than 48 corner vertices.\n", i);
            pts_t.emplace_back(tmp_t);
        }
        else{
            cv::find4QuadCornerSubpix(dst2, tmp_t, cv::Size(5, 5));
            pts_t.emplace_back(tmp_t);
        }
    }
}

int main(int, char **){
    cv::setNumThreads(16);              // 线程上限 16

    int valid_cnt = 0;

    ///=========================== 已经标定的摄像头参数 =============================///
    std::vector<cv::Point3f> k1 = std::vector<cv::Point3f>{
    	cv::Point3f(1808.70640125070,	0, 744.352105132516),
    	cv::Point3f(0, 1812.56192444507, 516.776530407297),
    	cv::Point3f(0, 0, 1.00)
    };
    cv::Mat insMb = cv::Mat(k1);
    insMb.convertTo(insMb, CV_64F);

    std::vector<cv::Point3f> k2 = std::vector<cv::Point3f>{
    	cv::Point3f(579.5235, 0, 313.9499),
    	cv::Point3f(0, 584.2775, 229.2345),
    	cv::Point3f(0, 0, 1.0000)
    };
    cv::Mat insMt = cv::Mat(k2);
    insMt.convertTo(insMt, CV_64F);

    std::vector<float> distCoeffs1=std::vector<float>{
    	-0.4413, 0.2410, 0.0071, 0.0021
    };

    std::vector<float> distCoeffs2=std::vector<float>{
    	-0.1189, 0.7223, -0.0039, -0.0081
    };
    ///=========================== ++++++++++++++ =============================///

    findCorners(1, image_num + 1);

    printf("Corner_b size: %lu, corner_t size: %lu\n", corner_b.size(), corner_t.size());
    for(int i = 0; i < image_num; ++i){
        if(pts_b[i].size() == 48 && pts_t[i].size() == 48){
            corner_b.emplace_back(pts_b[i]);
            corner_t.emplace_back(pts_t[i]);
            ++valid_cnt;
        }
    }

    //cv::drawChessboardCorners;
    worldPoints(world_pts, 40, x_num, y_num, valid_cnt);
    printf("Calibrating stereo-camera....\n");
    printf("Calibrating with %d sets of point data.\n", valid_cnt);

    cv::stereoCalibrate(world_pts, corner_b, corner_t, insMb,
            distCoeffs1, insMt, distCoeffs2, cv::Size(1440, 1080), R, T, E, F, 256,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-6));
    
    printf("Process completed.\n");

    printf("R matrix: \n");
    printMat(R);

    printf("T matrix: \n");
    printMat(T);

    printf("E matrix: \n");
    printMat(E);

    printf("F matrix: \n");
    printMat(F);

    //printf("Re-calculation for size(640, 480)....\n\n");
    //cv::stereoCalibrate(world_pts, corner_b, corner_t, insMb,
    //        distCoeffs1, insMt, distCoeffs2, cv::Size(640, 480), R, T, E, F, 256,
    //        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));
    //
    //printf("Process completed.\n");
    //
    //printf("R matrix: \n");
    //printMat(R);
    //
    //printf("T matrix: \n");
    //printMat(T);
    //
    //printf("E matrix: \n");
    //printMat(E);
    //
    //printf("F matrix: \n");
    //printMat(F);

    return 0;
}