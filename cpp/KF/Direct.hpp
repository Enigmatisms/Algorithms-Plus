#ifndef __DIRECT__
#define __DIRECT__

// 多层直接法相机位姿估计
#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

// Ceres 误差函数设定
class ErrorTerm{
public:

    /// @brief 构建误差函数项
    /// @note 注意此处的p不是原坐标，需要经过尺度变换
    ErrorTerm(
        const cv::Mat& _t,
        const Eigen::Matrix3d& _K,
        const Eigen::Matrix3d& _Kinv,
        const Eigen::Vector3d& p, 
        double val, double _d, int _pyr
    ):
        tar(_t), K(_K), Kinv(_Kinv), p1(p), pixel_val(val), depth(_d), pyr(_pyr) 
    {;}

    /**
     * @brief 残差函数
     * @param qr 旋转量 （四元数）
     * @param t 平移量
     * @param residual 为残差
     * @note 最垃圾的方法就是自己把四元数参数取出来，按数学公式计算了
     * TODO: review 是否能有效求导?(尤其是那个getPixel这个位置，让我觉得很虚)
     */
    template<typename T>
    bool operator()(const T* const qr, const T* t,  T* residual) const{
        Eigen::Map<const Eigen::Quaternion<T> > _qr(qr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> _t(t);

        Eigen::Quaternion<T> q_inv = _qr.conjugate();

        // 此处应该存在一个scale因子，最后再除以一个scale因子
        // 多层直接法的一个问题是，上一层的最终结果如何使用到这一层上
        // 对于光流，图像上的dx, dy显然是可以直接尺度变换的，但是这个问题不太一样
        // 感觉直接尺度变换就好了
        double scale = pow(2, 2 - pyr);
        Eigen::Vector3d temp_p1(p1(0) * scale, p1(1) * scale, 1);
        Eigen::Matrix<T, 3, 1> p2_cam = q_inv * (depth * Kinv * temp_p1 * pow(2, 2 - pyr) - _t);
        Eigen::Matrix<T, 3, 1> p2 = K * p2_cam / p2_cam(2);
        // 变回到对应金字塔下
        p2(0) /= scale;                    
        p2(1) /= scale;
        residual[0] = pow(val - getPixel(p2), 2);       // 个人觉得这一步很可能出问题（感觉函数没办法往下传递了）
        // 底下的floor函数又实际上是非线性的 导数会和图像的梯度有关，我觉得这个可能做不出来
        return true;
    }

    static ceres::CostFunction* Create(
        const cv::Mat& _t,
        const Eigen::Matrix3d& _K,
        const Eigen::Matrix3d& _Kinv,
        const Eigen::Vector3d& p, 
        double val, double _d, int _pyr
    ){
        return new ceres::AutoDiffCostFunction<ErrorTerm, 1, 4, 3>(
            new ErrorTerm(_t, _K, _Kinv, p, val, _d, _pyr)
        );
    }
private:
    template<typename T>
    double getPixel(Eigen::Matrix<T, 3, 1>& pos) const{
        if (pos(0) < 0){
            pos(0) = 0;
        }
        if (pos(0) > tar.cols - 1){
            pos(0) = tar.cols - 1;
        }
        if (pos(1) < 0){
            pos(1) = 0;
        }
        if (pos(1) > tar.rows - 1){
            pos(1) = tar.rows;
        }
        uchar *data = &tar.data[int(pos(1)) * tar.step + int(pos(0))];
        double& xx = pos(0) - floor(pos(0));
        double& yy = pos(1) - floor(pos(1));
        // 插值
        return double(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[tar.step] +
            xx * yy * data[tar.step + 1]
        );
    }
private:
    const Eigen::Matrix3d& K;       // 相机参数
    const Eigen::Matrix3d& Kinv;    // 反投影需要使用
    const cv::Mat& tar;             // 目标平面下的图像
    Eigen::Vector3d p1;             // 相机1下某个坐标点的像素齐次化坐标
    double pixel_val;               // reference图像下的像素坐标值
    double depth;                   // 参考图像下点的深度
    int pyr;                        // 金字塔层数 与尺度变换有关
};

class Direct{
public:
    Direct(const Eigen::Matrix3d& _K):
        K(_K)
    {
        Kinv = K.inverse();
    }
    ~Direct(){;}
public:
    void drawResult(cv::Mat& src);          // 绘制结果
    void optimization(const double ** const d_map, int size);    // 优化相机位姿
private:
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

    /// TODO: 插值需要进行判定：深度为0的位置将不进行插值
    /// 所有插值点都为零则放弃这个参数块
    double depthMapInterpolation(const double const **arr, double x, double y, int rows, int cols){
        x = cv::max(0.0, x);
        x = cv::min((double)(rows - 1), x);
        y = cv::max(0.0, y);
        y = cv::min((double)(cols - 1), y);
        const double const *data = &arr[int(y)][int(x)];
        double xx = x - floor(x);
        double yy = y - floor(y);
        // 插值
        return double(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[cols] +
            xx * yy * data[cols + 1]
        );
    }
private:
    std::vector<cv::KeyPoint> pts;
    std::vector<cv::Mat> pyramid;       // 金字塔结构
    std::vector<cv::Mat> target;        // 金字塔结构
    const Eigen::Matrix3d& K;
    Eigen::Matrix3d Kinv;
};

void Direct::optimization(const double ** const depth_map, int size){

    // 极其壮观的四层for循环
    double initial[7] = {0, 0, 0, 1, 0, 0, 0};
    for (int pyr = 0; pyr < 3; pyr++){
        ceres::Problem prob;
        for (size_t p = 0; p < pts.size(); p++){
            auto pt = pts[p].pt;
            int rows = target[pyr].rows;
            int cols = target[pyr].cols;
            for (int i = -size; i <= size; i++){
                for (int j = -size; j <= size; j++){
                    if (pt.x + i >= 0 && pt.x + i <= cols - 1 &&
                        pt.y + j >= 0 && pt.y + j <= rows - 1)
                    {
                        Eigen::Vector3d pixel(pt.x, pt.y, 1);
                        ceres::CostFunction* cost_func = ErrorTerm::Create(target[pyr], K, Kinv, pixel,
                            getPixel(pyramid[pyr], pt.x + i, pt.y + j), depthMapInterpolation(
                            depth_map, pt.x + i, pt.y + j, rows, cols), pyr
                        );

                        /// TODO: review 初值传递问题, 是否能这样进行初值传递? 尺度变换问题解决了吗
                        prob.AddResidualBlock(cost_func, nullptr, initial);
                    }
                }
            }
        }
        ceres::Solver::Options opts;
        opts.linear_solver_type = ceres::DENSE_QR;
        opts.minimizer_progress_to_stdout = false;
        opts.max_linear_solver_iterations = 15;
        opts.function_tolerance = 10e-5;
        ceres::Solver::Summary summary;

        ceres::Solve(opts, &prob, &summary);
    }
    // 最后的结果输出在此处
}

#endif // __DIRECT__

