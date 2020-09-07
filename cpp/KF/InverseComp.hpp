#ifndef __INVERSE_COMP__
#define __INVERSE_COMP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>

// 逆向多层光流估计 此处无需使用合成法（没有更多可以预先计算的近似了）
// 没有使用并行加速优化
class InverseAdd{
public:
    InverseAdd(){;}
    ~InverseAdd(){;}
public:
    /// @brief 输入一张图像
    void inputImage(const cv::Mat& src, const cv::Mat& dst);

    /// @brief 查看金字塔绘制等情况
    void debugDisplay(const cv::Mat& src, const cv::Mat& dst);

    /// @brief 将光流估计结果绘制在dst上
    void drawResult(cv::Mat& dst, bool debug = false) const;

    /// @brief 光流计算
    void calcOpticalFlow(int size = 4, int max_iter = 10);

    double getPixel(const cv::Mat &img, double x, double y) const{
        x = cv::max(0.0, x);
        x = cv::min((double)(img.cols - 1), x);
        y = cv::max(0.0, y);
        y = cv::min((double)(img.rows - 1), y);
        uchar *data = &img.data[int(y) * img.step + int(x)];
        double xx = x - floor(x);
        double yy = y - floor(y);
        // 插值
        return double(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
        );
    }

    // X 轴正方向
    double getPixelGradX(const cv::Mat& src, double i, double j) const{
        return 0.5 * (getPixel(src, i + 1, j) - getPixel(src, i - 1, j));
    }

    // Y 轴正方向
    double getPixelGradY(const cv::Mat& src, double i, double j) const{
        return 0.5 * (getPixel(src, i, j + 1) - getPixel(src, i, j - 1));
    }
private:
    // 三层光流估计
    std::vector<cv::Mat> old;
    std::vector<cv::Mat> now;
    std::vector<cv::KeyPoint> estimates;        // 上一层的估计
    std::vector<Eigen::Vector2d> delta;             // 每个特征点的光流估计
    std::vector<bool> tracked;                  // 是否被持续跟踪

    const cv::Scalar GREEN = cv::Scalar(255);
};

void InverseAdd::inputImage(const cv::Mat& src, const cv::Mat& dst){
    for (int pyr = 0; pyr < 3; pyr++){
        cv::Mat t1, t2;
        cv::resize(src, t1, cv::Size(src.cols / pow(2, 2 - pyr), src.rows / pow(2, 2 - pyr) ) );
        old.push_back(t1);
        cv::resize(dst, t2, cv::Size(dst.cols / pow(2, 2 - pyr), dst.rows / pow(2, 2 - pyr) ) );
        now.push_back(t2);
    }
    /// TODO: 这里的threshold有待讨论
    cv::FAST(src, estimates, 120);

    tracked.resize(estimates.size());
    delta.resize(estimates.size());
    for (size_t i = 0; i < estimates.size(); i++){
        tracked[i] = false;
    }
}


/// 此处使用了反向叠加法
/// 反向法判定模板在如何运动时，可以与原图进行更好的重合
/// 比如模板上的一个patch，移动dx, dy可以与原图进行重合，那么相当于原图移动-dx, -dy与模板重合
/// 使用梯度进行近似，模板梯度近似当前图梯度
void InverseAdd::calcOpticalFlow(int size, int max_iter){
    std::vector<Eigen::Vector2d> Jacs;      // 逆向光流法所取的梯度是模板函数的梯度 迭代过程中不变
    Eigen::Matrix2d Hes = Eigen::Matrix2d::Zero();
    for (int pyr = 0; pyr < 3; pyr ++){
        for (size_t p = 0; p < estimates.size(); p++){
            if (tracked[p] == false){       // 在上一层已经追踪失败了 本层若需要继续追踪则需要reset
                delta[p].setZero();         // 重新进行估计
                tracked[p] = true;
            }
            else{
                if (pyr > 0){                 
                    delta[p] *= 2;
                }
            }
            // 对于一个特征点
            auto pt = estimates[p].pt / pow(2, 2 - pyr);    // 特征点的位置尺度变换
            double old_cost = INFINITY, error = 0.0, cost = 0.0;
            Jacs.clear();
            Hes = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            for (int it = 0; it < max_iter; it ++){
                b = Eigen::Vector2d::Zero();
                cost = 0.0;
                
                int counter = 0;
                for (int i = -size; i <= size; i++){
                    for (int j = -size; j <= size; j ++){
                        if (it == 0){
                            // 图像梯度
                            Jacs.emplace_back(
                                getPixelGradX(old[pyr], pt.x + i, pt.y + j),
                                getPixelGradY(old[pyr], pt.x + i, pt.y + j)
                            );
                            // 使用Hessian矩阵的近似
                            Hes += Jacs.back() * Jacs.back().transpose();
                        }

                        /// delta放在now或者old下实际上是一样的（相对运动）
                        /// 当前图像 与 历史图像 之间的比较 （当前 - 历史） 为error
                        error = getPixel(now[pyr], pt.x + delta[p](0) + i, pt.y + delta[p](1) + j) - 
                            getPixel(old[pyr], pt.x + i, pt.y + j);

                        /// 高斯牛顿法，对error函数对应的 0.5||error + JacobianT * delta||^2 进行最小化
                        b += - error * Jacs[counter];
                        counter++;
                        cost += pow(error, 2);
                    }
                }

                Eigen::Vector2d update = Hes.ldlt().solve(b);
                // 出现了无解的情况 对于此特征点p的追踪失败了
                if (std::isnan(update[0]) == true){
                    tracked[p] = false;
                    break;  
                }
                if (cost >= old_cost){
                    break;
                }
                old_cost = cost;
                delta[p] += update;          // 更新update
            }
        }
    }
}   

void InverseAdd::debugDisplay(const cv::Mat& src, const cv::Mat& dst){
    inputImage(src, dst);
    int cnt = 0;
    for (const cv::Mat& img: old){
        char path[40];
        snprintf(path, 40, "/home/sentinel/testPics/old%d.png", cnt++);
        cv::imwrite(path, img);
    }
    cnt = 0;
    for (const cv::Mat& img: now){
        char path[40];
        snprintf(path, 40, "/home/sentinel/testPics/now%d.png", cnt++);
        cv::imwrite(path, img);
    }
    for (const cv::KeyPoint& vec: estimates){
        std::cout << vec.pt.x << ", " << vec.pt.y << std::endl;
    }
}

void InverseAdd::drawResult(cv::Mat& dst, bool debug) const{
    for (size_t i = 0; i < estimates.size(); i++){
        cv::Point2f end_point = estimates[i].pt;
        if (debug == false){
            end_point += cv::Point2f(delta[i](0), delta[i](1));
        }
        else{
            end_point += cv::Point2f(8, 8);
        }
        const cv::Point2f& start_point = estimates[i].pt;
        cv::circle(dst, end_point, 4, GREEN, -1);
        cv::line(dst, start_point, end_point, GREEN, 1);
    }
}

#endif // __INVERSE_COMP__

