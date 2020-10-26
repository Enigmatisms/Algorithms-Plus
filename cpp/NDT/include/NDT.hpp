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

// 误差函数定义
class ErrorTerm{
public:
    ErrorTerm(const cv::Point2d& pt, const Eigen::Vector2d& _mu, const Eigen::Matrix2d& _icov):
        mu(_mu), icov(_icov), x(pt.x), y(pt.y)
    {
        ;
    }
    ~ErrorTerm(){;}
public:
    // 最大PDF误差函数定义 （最小化负对数）
    template<typename T>
    bool operator()(const T* const param, T* residual) const{
        T _x = cos(param[2]) * (T)x - sin(param[2]) * (T)y + param[0];
        T _y = sin(param[2]) * (T)x + cos(param[2]) * (T)y + param[1];
        Eigen::Matrix<T, 2, 1> vec(_x, _y);
        vec = vec - mu;
        auto product = - vec.transpose() * icov * vec;
        residual[0] = - product.value();
        return true;
    }

    // 创建误差函数
    static ceres::CostFunction* Create(
        const cv::Point2d& pt,
        const Eigen::Vector2d& _mu,
        const Eigen::Matrix2d& _icov
    ){
        return new ceres::AutoDiffCostFunction<ErrorTerm, 1, 3>(
            new ErrorTerm(pt, _mu, _icov)
        );
    }
private:
    const Eigen::Vector2d& mu;
    const Eigen::Matrix2d& icov;        // 协方差矩阵的逆
    double x;
    double y;
};

// 新误差策略：点云不动，调整高斯分布的均值与方差，使得负对数最大
class Error{
public:
    Error(const cv::Point2d& pt){
        x = pt.x;
        y = pt.y;
    }
    ~Error();
public:
    // 参数应该是：平移 * 2 (均值) 三个参数（协方差矩阵的逆中的三个参数（对称））
    template<typename T>
    bool operator()(const T* const u, const T* const covp, T* residual) const{
        Eigen::Matrix<T, 2, 2> icov;
        icov << covp[0], covp[2], covp[2], covp[1];
        Eigen::Matrix<T, 2, 1>  vec;
        vec << x - u[0], y - u[1];
        auto product = - vec.transpose() * icov * vec;

        // T _x = cos(param[2]) * (T)x - sin(param[2]) * (T)y + param[0];
        // T _y = sin(param[2]) * (T)x + cos(param[2]) * (T)y + param[1];
        // Eigen::Matrix<T, 2, 1> vec(_x, _y);
        // vec = vec - mu;
        // auto product = - vec.transpose() * icov * vec;
        // residual[0] = - product.value();
        return true;
    }
private:
    double x;
    double y;
};

class NDT{
public:
    NDT(){
        mu = Eigen::Vector2d(X_SIG, Y_SIG);
        cov << X_SIG * X_SIG, 0, 0, Y_SIG * Y_SIG;
        rng = new cv::RNG(time(NULL));
    }
    ~NDT(){
        delete rng;
    }
public:
    void initDistribute(cv::Mat& src);    // 生成初始分布（得到初始分布的一堆点并且绘图）

    /// 实现的时候需要注意，我们随机地生成角度 / 平移量 记录在参数 a mov中
    /// 在对初始点云进行变换的时候，角度 / 平移量还需要添加高斯随机数噪声，并且还需要随机投点（噪点）
    /// 得到最后的点云为pts 我们优化pts
    void getPerturbSim(cv::Mat& src, std::vector<cv::Point2d>& pts, double& a, cv::Point2d& mov, double sig = 1);  // 得到添加噪声后的仿真分布点云（当作我们的灯条)
    
    /// 求解位姿变换（如何从getPerturb点云匹配到初始点云？），并绘制变换后的点云
    void minimize(const std::vector<cv::Point2d>& pts, cv::Mat& src, double init_angle);

    void readAndConvert(std::string path, std::vector<cv::Point2d>& pts);
private:
    /// 计算以(X_SIG, Y_SIG)为中心，(X_SIG, Y_SIG)为两个方向的标准差，两个方向独立的高斯分布PDF
    static double gauss2d(double x, double y){
        return exp(- ((x - X_SIG) * (x - X_SIG) / (X_SIG * X_SIG)
            + (y - Y_SIG) * (y - Y_SIG) / (Y_SIG * Y_SIG)) / 2
        );
    }
private:
    std::vector<cv::Point2d> init_pts;          // 初始点云
    Eigen::Vector2d mu;                         // 均值
    Eigen::Matrix2d cov;                        // 协方差
    std::mutex mtx;
    cv::RNG* rng;
};


#endif  //__NDT_HPP__