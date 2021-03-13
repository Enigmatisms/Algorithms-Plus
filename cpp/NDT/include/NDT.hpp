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


class NDT{
public:
    NDT(){;}
    ~NDT(){;}
public:
    void lightMatching(cv::Mat& src1, cv::Mat& src2, int number, double sig_c = 0.25);

    void lightDiffusion(cv::Mat& src1, cv::Mat& src2, int number, double radius);

    template<typename T>
    void drawDiffusion(cv::Mat& src, const T* const _top, const T* const _ctr, T radius) const; 
private:
    cv::Mat readAndConvert(ptsType& pts, int number) const;

    void readAndConvert(std::vector<double>& pts, cv::Mat& dst, int number) const;

    void initParamEstimate(const ptsType& pts, double* u, double* covp) const;

    void drawGaussian(cv::Mat& src, const Eigen::Vector2d& mu, const Eigen::Matrix2d& cov, double sig);       // 二维高斯分布绘制

    void saveToFile(const cv::Mat& src1, const cv::Mat& src2, int number) const;

    void betterInitialize(const cv::Mat& src, double* _top, double* _ctr) const;
private:
    const std::string prefix = "/home/sentinel/Dataset/light/";     // 路径前缀
};

#endif  //__NDT_HPP__