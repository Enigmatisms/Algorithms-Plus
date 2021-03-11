/**
 * @author hqy
 * @date 2020.10.25
 * NDT灯条参数估计
 */
#ifndef __NDT_HPP__
#define __NDT_HPP__
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include "ErrorTerm.hpp"

#define X_SIG 15.0
#define Y_SIG 60.0
#define TRANS_X 100.0
#define TRANS_Y 50.0
#define ANGLE 0.8

class NDT{
public:
    NDT(){;}
    ~NDT(){;}
public:
    void lightMatching(cv::Mat& src1, cv::Mat& src2, int number, double sig_c = 0.25);

    void lightDiffusion(cv::Mat& src1, cv::Mat& src2, int number);

    template<typename T>
    void drawDiffusion(cv::Mat& src, const T* const _top, const T* const _ctr, const T* const _dec) const; 
private:
    cv::Mat readAndConvert(ptsType& pts, int number) const;

    void readAndConvert(std::vector<double>& pts, cv::Mat& dst1, cv::Mat& dst2, int number) const;

    void diffusionEstimate(const cv::Mat& src1, double* top, double* ctr, double* pars);

    void initParamEstimate(const ptsType& pts, double* u, double* covp) const;

    void drawGaussian(cv::Mat& src, const Eigen::Vector2d& mu, const Eigen::Matrix2d& cov, double sig);       // 二维高斯分布绘制

    void saveToFile(const cv::Mat& src1, const cv::Mat& src2, int number) const;

    /// 计算以(X_SIG, Y_SIG)为中心，(X_SIG, Y_SIG)为两个方向的标准差，两个方向独立的高斯分布PDF
    static double gauss2d(double x, double y){
        return exp(- ((x - X_SIG) * (x - X_SIG) / (X_SIG * X_SIG)
            + (y - Y_SIG) * (y - Y_SIG) / (Y_SIG * Y_SIG)) / 2
        );
    }
private:
    const std::string prefix = "/home/sentinel/Dataset/light/";     // 路径前缀
};

#endif  //__NDT_HPP__