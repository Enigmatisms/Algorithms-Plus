#ifndef __DIRECT__
#define __DIRECT__

/// 多层直接法相机位姿估计
/// @author Sentinel
/// @date 2020.9.7
/// @note 不稳定
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
#include <ceres/cubic_interpolation.h>

const cv::Scalar GREEN(0, 255, 0);

using CeresInterpolator = const ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 1> >&;

template<typename T>
void printVec(const T& vec, int size){
    for (int i = 0; i < size - 1; i++){
        std::cout << vec(i) << ", ";
    }
    std::cout << vec(size - 1) << std::endl;
}

// 输出方形矩阵
template<typename T>
void printMat(const T& mat, int size){
    for (int i = 0; i < size; i++){
        for (int j = 0; j < size - 1; j++){
            std::cout << mat(i, j) << ", ";
        }
        std::cout << mat(i, size - 1) << std::endl;
    }
}

double depthMapInterpolation(const float* arr, double x, double y, int rows, int cols){
    if (x >= cols || x < 0) return 0;
    if (y >= rows || y < 0) return 0;
    const float* data = &arr[int(y) * cols + int(x)];
    double xx = x - floor(x);
    double yy = y - floor(y);
    // 存在一个深度无法估计点就直接退出
    if (data[0] == 0 || data[1] == 0 || data[cols] == 0 || data[cols + 1] == 0){
        return 0;
    }
    return double(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[cols] +
        xx * yy * data[cols + 1]
    );
}

// Ceres 误差函数设定
class ErrorTerm{
public:
    /// @brief 构建误差函数项
    /// @note 注意此处的p不是原坐标，需要经过尺度变换
    ErrorTerm(
        const Eigen::Matrix3d& _K,
        const Eigen::Matrix3d& _Kinv,
        CeresInterpolator itp,
        Eigen::Vector3d p, 
        double val, double _d
    ):
        K(_K), Kinv(_Kinv), interp(itp), p1(p), pixel_val(val), depth(_d)
    {;}

    /**
     * @brief 残差函数
     * @param qr 旋转量 （四元数）
     * @param t 平移量
     * @param residual 为残差
     */
    template<typename T>
    bool operator()(const T* const qr, const T* t,  T* residual) const{
        Eigen::Map<const Eigen::Quaternion<T> > _qr(qr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > _t(t);
        // Eigen::Quaternion<T> q_inv = _qr.conjugate();

        // 不乘以T(1)将会报错 输入的点是对应金字塔层数下的特征点（齐次坐标值，无需进行尺度变换）
        Eigen::Matrix<T, 3, 1> temp_p1(p1(0) * T(depth), p1(1) * T(depth), (T)depth);   // 将特征点变换回原坐标下
        Eigen::Matrix<T, 3, 1> p2_cam = _qr * Kinv * temp_p1 + _t;  
        Eigen::Matrix<T, 3, 1> p2 = K * p2_cam / p2_cam(2);         // 得到最底层金字塔下的p2坐标

        T gray_val;
        // 此处对对应金字塔层数的图片插值函数进行evaluate 输入 先行后列
        interp.Evaluate(p2(1), p2(0), &gray_val);                   // 得到插值结果(在此之前需要进行一个判定？是否超界)
        residual[0] =  - gray_val + (T)pixel_val;       
        return true;
    }

    static ceres::CostFunction* Create(
        const Eigen::Matrix3d& _K,
        const Eigen::Matrix3d& _Kinv,
        CeresInterpolator itp,
        Eigen::Vector3d p, 
        double val, double _d
    ){
        return new ceres::AutoDiffCostFunction<ErrorTerm, 1, 4, 3>(
            new ErrorTerm(_K, _Kinv, itp, p, val, _d)
        );
    }
private:
    const Eigen::Matrix3d& K;       // 相机参数
    const Eigen::Matrix3d& Kinv;    // 反投影需要使用
    CeresInterpolator interp;       // 图像函数
    Eigen::Vector3d p1;             // 相机1下某个坐标点的像素齐次化坐标
    double pixel_val;               // reference图像下的像素坐标值
    double depth;                   // 参考图像下点的深度
};

class Direct{
public:
    Direct(const Eigen::Vector4d& _intr, int _pyr_num = 3):
        intr(_intr), pyr_num(_pyr_num)
    {}
    ~Direct(){;}
public:
    void inputImages(const cv::Mat& p1, const cv::Mat& p2);     // 输入两张图片
    void optimization(const std::vector<cv::Mat>& depths, int size);   // 优化相机位姿
    void drawResult(cv::Mat& src, const float * const d_map);
    void debugDraw(cv::Mat& src);
    void randomSelector(const cv::Mat& src, std::vector<cv::Point2f>& vertices, int num = 60);
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

    static Eigen::Vector3d Point2Eigen(const cv::Point& p){
        Eigen::Vector3d vec(p.x, p.y, 1.0);
        return vec;
    }
private:
    std::vector<std::vector<cv::Point2f> > pts;
    std::vector<cv::Mat> pyramid;       // 金字塔结构
    std::vector<cv::Mat> target;        // 金字塔结构
    Eigen::Quaterniond rotate;
    Eigen::Vector3d trans;
    const Eigen::Vector4d& intr;
    int pyr_num;
};

void Direct::optimization(const std::vector<cv::Mat>& depths, int size){
    // 进行位姿优化
    rotate.setIdentity();
    trans.setZero();
    for (int pyr = 0; pyr < pyr_num; pyr++){
        double scale = pow(2, pyr_num - 1 - pyr);
        const cv::Mat& img = target[pyr];
        // 构造目标图像插值函数
        ceres::Grid2D<uchar, 1> temp(img.data, 0, img.rows, 0, img.cols);
        ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 1> > _itp(temp);
        ceres::Problem prob;    // 构建优化问题 并设置四元数维度
        ceres::LocalParameterization* quaternion = new ceres::EigenQuaternionParameterization;

        // 计算新的内参矩阵（当图像缩小为1/scale时，主点以及x,y轴方向放大率也会变为1/scale倍）
        Eigen::Matrix3d K;
        K << intr(0) / scale, 0, intr(1) / scale,
            0, intr(2) / scale, intr(3) / scale,
            0, 0, 1;
        Eigen::Matrix3d Kinv = K.inverse();
        for (size_t p = 0; p < pts.size(); p++){
            const cv::Point2f& pt = pts[pyr][p];  //原特征点尺度变换
            int rows = img.rows;
            int cols = img.cols;
            for (int i = -size; i <= size; i++){
                for (int j = -size; j <= size; j++){
                    if (pt.x + i >= 0 && pt.x + i <= cols - 1 &&
                        pt.y + j >= 0 && pt.y + j <= rows - 1)
                    {
                        double depth = depthMapInterpolation((float *)depths[pyr].data, pt.x + i, pt.y + j, rows, cols);
                        if (depth <= 0.1){         // 深度的单位是毫米，小于20cm的深度一律认为是无效的（深度图中存在无法确定的位置）
                            continue;
                        }
                        Eigen::Vector3d pixel(pt.x + i, pt.y + j, 1);
                        ceres::CostFunction* cost_func = ErrorTerm::Create(K, Kinv, _itp,
                            pixel, getPixel(pyramid[pyr], pt.x + i, pt.y + j), depth
                        );
                        prob.AddResidualBlock(cost_func, nullptr, rotate.coeffs().data(), trans.data());
                    }
                }
            }
        }
        prob.SetParameterization(rotate.coeffs().data(), quaternion);
        ceres::Solver::Options opts;
        opts.linear_solver_type = ceres::DENSE_QR;
        opts.minimizer_progress_to_stdout = true;
        opts.max_linear_solver_iterations = 100;
        opts.function_tolerance = 1e-6;
        ceres::Solver::Summary summary;
        ceres::Solve(opts, &prob, &summary);
        std::cout << summary.FullReport() << std::endl;
        std::cout << "=========================Pyramid:" << pyr << "=======================\n";
        std::cout << "Rotation result:\n";
        printVec<Eigen::Vector4d>(rotate.coeffs(), 4);
        std::cout << "Translation result:\n";
        printVec<Eigen::Vector3d>(trans, 3);
    }
    std::cout << "Optimization step is completed.\n";
}

void Direct::randomSelector(const cv::Mat& src, std::vector<cv::Point2f>& vertices, int num){
    cv::RNG rng;
    for (int i = 0; i < num; i++){
        float x = rng.uniform(10, src.cols - 10);
        float y = rng.uniform(10, src.rows - 10);
        vertices.emplace_back(x, y);
    }
}

void Direct::inputImages(const cv::Mat& p1, const cv::Mat& p2){
    int rows = p1.rows;
    int cols = p1.cols;
    pts.resize(pyr_num);
    for (int pyr = 0; pyr < pyr_num - 1; pyr ++){
        cv::Mat temp1, temp2;
        // 构建金字塔 并对每层的特征点进行提取
        cv::resize(p1, temp1, cv::Size((int)(cols / pow(2, pyr_num - 1 - pyr)), (int)(rows / pow(2, pyr_num - 1 - pyr))));
        cv::resize(p2, temp2, cv::Size((int)(cols / pow(2, pyr_num - 1 - pyr)), (int)(rows / pow(2, pyr_num - 1 - pyr))));
        randomSelector(temp1, pts[pyr], 600);
        pyramid.push_back(temp1);
        target.push_back(temp2);
    }
    pyramid.emplace_back(p1);
    target.emplace_back(p2);
    randomSelector(p1, pts[pyr_num - 1], 600);
}

void Direct::drawResult(cv::Mat& src, const float * const d_map){
    Eigen::Matrix3d K;
    K << intr(0), 0, intr(1),
        0, intr(2), intr(3),
        0, 0, 1;
    Eigen::Matrix3d Kinv = K.inverse();
    for (const cv::Point2f& pt: pts.back()){
        double depth = depthMapInterpolation(d_map, pt.x, pt.y, src.rows, src.cols);
        if (depth > 0.1){
            cv::Point start(pt.x, pt.y);
            cv::circle(src, start, 4, GREEN, -1);
            Eigen::Vector3d _p = Point2Eigen(pt);
            Eigen::Vector3d p2_cam = rotate.conjugate() * (depth * Kinv * _p - trans);  // P的相机2相机坐标位置
            Eigen::Vector3d p2 = K * p2_cam / p2_cam(2);                                // 像素坐标位置
            cv::Point dest(p2(0), p2(1));
            cv::circle(src, dest, 2, GREEN, -1);
            cv::line(src, start, dest, GREEN, 2);
        }   
    }
}

void Direct::debugDraw(cv::Mat& src){
    std::cout << "FAST feature point number: " << pts.size() << std::endl;
    for (size_t i = 0; i < pts.back().size(); i++){
        const cv::Point2f& pt = pts[i].back();
        cv::circle(src, cv::Point(pt.x, pt.y), 8, GREEN, -1);
    }
}

#endif // __DIRECT__

