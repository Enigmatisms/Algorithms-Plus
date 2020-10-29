/**
 * @author hqy
 * @date 2020.10.25
 * NDT灯条参数估计
 * 明确一下问题：
 *      对于粗匹配的的灯条选框，首先进行一次TO_ZERO_INV的阈值化，保存那些值较大的灯条（灰度图）
 *      对每一个存在值的像素（使用for_each）保存其中心位置（可能存在噪点）
 *      从本灯条影射到初始分布，存在一个位置的（平移），旋转的（单个参数），伸缩的变换
 *      假设先不考虑伸缩变换，那么2D点云仿真（1.从初始分布得到一个点群 点群里投入随机的噪声，点群进行旋转（每个点的旋转/平移存在细小的扰动）平移）
 *      初始灯条估计已经能得到一个较好的值了，那么我们仿真时，移动不需要过大。
 *          优化函数的定义：需要优化每个变换到初始分布上的点的概率密度之和（1.cost函数本身不能有过于严重的数值问题）(2.原来含有outliers的数据需要鲁棒核)
 *          那个优化函数需要推导一下（由于存在一个概率值归一化的问题以及外点数量估计的问题）
 *          个人觉得这个可以简单一些，直接使用0.9995 * 原概率密度函数 + 0.0005(选择一个初始的参数，或者大一些)
 *          那么每一个点对应一个Ceres::Residual Block
 */
#ifndef __NDT_HPP__
#define __NDT_HPP__
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <mutex>

#define X_SIG 15.0
#define Y_SIG 60.0
#define TRANS_X 100.0
#define TRANS_Y 50.0
#define ANGLE 0.8

#define print_var(x) (std::cout << #x << ": " << x << std::endl)

// 新误差策略：点云不动，调整高斯分布的均值与方差，使得负对数最大
class Error{
public:
    Error(const std::vector<cv::Point3d>& _pts, double sig_c = 0.25):
        pts(_pts)
    {
        sigma_coeff = sig_c;
    }
    ~Error(){;}
public:
    // 参数应该是：平移 * 2 (均值) 三个参数：矩阵的方差 / 矩阵对角协方差     | a b |
    // a = covp[0], c = covp[1], b = covp[2]                        | b c |
    template<typename T>
    bool operator()(const T* const u, const T* const icovp, T* residual) const{
        Eigen::Matrix<T, 2, 2> icov;
        icov << icovp[0], icovp[2], icovp[2], icovp[1];
        T res(0);
        for (const cv::Point3d& pt: pts) {
            Eigen::Matrix<T, 2, 1> vec(T(pt.x) - u[0], T(pt.y) - u[1]);
            T product = - T(vec.transpose() * icov * vec) / T(2);
            res += ceres::exp(product) * (T)pt.z;
        }
        print_var(res);
        residual[0] = ceres::sqrt(1e3 - res);
        return true;
    }

    static ceres::CostFunction* Create(const std::vector<cv::Point3d>& _pts, double sig_c = 0.25){
        return new ceres::AutoDiffCostFunction<Error, 1, 2, 3>(
            new Error(_pts, sig_c)
        );
    }
private:
    const std::vector<cv::Point3d>& pts;
    double sigma_coeff;
};

class NDT{
public:
    NDT(){;}
    ~NDT(){;}
public:
    cv::Mat readAndConvert(std::vector<cv::Point3d>& pts, int number, uchar thresh = 60);

    void lightMatching(cv::Mat& src1, cv::Mat& src2, int number, double sig_c = 0.25);

    /// 估计初始值
    void initParamEstimate(const std::vector<cv::Point3d>& pts, double* u, double* covp) const;

    void drawGaussian(cv::Mat& src, const Eigen::Vector2d& mu, const Eigen::Matrix2d& cov);       // 二维高斯分布绘制
private:
    /// 计算以(X_SIG, Y_SIG)为中心，(X_SIG, Y_SIG)为两个方向的标准差，两个方向独立的高斯分布PDF
    static double gauss2d(double x, double y){
        return exp(- ((x - X_SIG) * (x - X_SIG) / (X_SIG * X_SIG)
            + (y - Y_SIG) * (y - Y_SIG) / (Y_SIG * Y_SIG)) / 2
        );
    }
private:
    std::mutex mtx;
    const std::string prefix = "/home/sentinel/light/";     // 路径前缀
};


#endif  //__NDT_HPP__