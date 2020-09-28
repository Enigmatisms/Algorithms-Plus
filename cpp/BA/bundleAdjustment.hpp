#ifndef __BA_ERRORTERM__HPP__
#define __BA_ERRORTERM__HPP__
#include "_common.hpp"
#include "ceres/ceres.h"
// #define USE_QUATERNION

class BAErrorTerm{
public:
    /// BALProblem中的 observations_就是我们需要使用的二维点
    BAErrorTerm(double _u, double _v):
        u(_u), v(_v)
    {
        ;
    }
    ~BAErrorTerm(){;};
public:

    /// @brief 残差定义
    /// 初始值是在AddResidualBlock阶段给出的 不用关心
    template <typename Ty>
    bool operator()(const Ty* const cams, const Ty* const pt, Ty* residual) const{
        #ifdef USE_QUATERNION
            Eigen::Quaternion<Ty> _R(cams[0], cams[1], cams[2], cams[3]);        // 相机旋转位姿
            Eigen::Matrix<Ty, 3, 1> _t(cams[4], cams[5], cams[6]);               // 相机平移
            // Ty focal = cams[7], k1 = cams[8], k2 = cams[9];                                         // 相机其他参数
        #else
            Eigen::Matrix<Ty, 3, 1> _vec(cams);                 // 相机旋转位姿
            Eigen::AngleAxis<Ty> _rvec(_vec.norm(), _vec.normalized());
            Eigen::Matrix<Ty, 3, 3> _R = _rvec.toRotationMatrix();
            Eigen::Matrix<Ty, 3, 1> _t(cams + 3);               // 相机平移
        #endif  //USE_QUATERNION
        Eigen::Matrix<Ty, 3, 1> pw(pt);                                          // 点云世界坐标
        ///========================== 建立观测方程（重投影）=============================
        const Ty _u = (Ty)u, _v = (Ty)v;
        Eigen::Matrix<Ty, 3, 1> P = _R * pw + _t;
        Eigen::Matrix<Ty, 3, 1> p = - P / P(2);
        Ty r2 = p(0) * p(0) + p(1) * p(1);
        #ifdef USE_QUATERNION
            Eigen::Matrix<Ty, 3, 1> proj = cams[7] * p * (Ty(1.0) + cams[8] * norm * norm + cams[9] * (Ty)pow(norm, 4));   // 畸变 + 投影
        #else
            Eigen::Matrix<Ty, 3, 1> proj = cams[6] * p * (Ty(1.0) + cams[7] * r2 + cams[8] * r2 * r2);   // 畸变 + 投影
        #endif  //USE_QUATERNION
        residual[0] = proj(0) - _u;
        residual[1] = proj(1) - _v;
    }

    /// 构造残差块，2表示残差是两维的 10 表示使用四元数定义时存在10个相机参数，3为相机对应点的空间位置参数
    /// 想法就是，遍历所有相机，对所有相机观测到的点进行遍历，每个相机-观测点对产生一个残差块，只需要考虑好如何得到其中一个残差块即可
    /// 在BALProblem中，我们有观测值，并且有每一个观测值对应的相机index, 并且在parameters中，相机参数之后都是初始点云的三位坐标
    /// 我们也存在着每一个obs对应点云中哪一个点 那么根据相机的参数/位姿 初始的三维坐标可以投影得到一个点，求重投影误差（两维残差即可）
    #ifdef USE_QUATERNION
    static ceres::CostFunction* Create(double _u, double _v){
        return new ceres::AutoDiffCostFunction<BAErrorTerm, 2, 10, 3>(
            new BAErrorTerm(_u, _v)
        );
    }
    #else
    static ceres::CostFunction* Create(double _u, double _v){
        return new ceres::AutoDiffCostFunction<BAErrorTerm, 2, 9, 3>(
            new BAErrorTerm(_u, _v)
        );
    }
    #endif  //USE_QUATERNION

private:
    double u;
    double v;
};

#endif  //__BA_ERRORTERM__HPP_