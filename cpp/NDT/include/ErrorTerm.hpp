#ifndef __ERROR_TERM_HPP
#define __ERROR_TERM_HPP
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <vector>
#include <utility>
#include <opencv2/core.hpp>

#define print_var(x) (std::cout << #x << ": " << x << std::endl)
using ptsType = std::vector<std::pair<double, cv::Point2d> >;

// ErrorTerm V3 (正态分布变换)
class Error{
public:
    Error(const ptsType& _v, double sig_c = 1):
        vex(_v)
    {
        sigma_coeff = sig_c;
    }
    ~Error(){;}
public:
    /// 注意，输入的完全是协方差的逆矩阵
    template<typename T>
    bool operator()(const T* const u, const T* const icovp, T* residual) const{
        Eigen::Matrix<T, 2, 2> icov;
        icov << icovp[0], icovp[2], icovp[2], icovp[1];
        T res(0);
        std::vector<T> res_keep(vex.size());
        Eigen::Matrix<T, 2, 1> pu(u[0], u[1]);
        #pragma omp parallel for num_threads(8)
        for (size_t i = 0; i < vex.size(); i++) {
            Eigen::Matrix<T, 2, 1> pos(vex[i].second.x, vex[i].second.y);
            Eigen::Matrix<T, 2, 1> vec = pos - pu;
            T prod = vec.transpose() * icov * vec;
            prod = - ceres::pow(prod, T(sigma_coeff));
            res_keep[i] = ceres::abs(ceres::exp(prod) - T(vex[i].first));
        }
        residual[0] = std::accumulate(res_keep.begin(), res_keep.end(), T(0.0));
        return true;
    }

    static ceres::CostFunction* Create(const ptsType& _v, double sig_c = 0.25){
        return new ceres::AutoDiffCostFunction<Error, 1, 2, 3>(
            new Error(_v, sig_c)
        );
    }
private:
    const std::vector<std::pair<double, cv::Point2d> >& vex;
    double sigma_coeff;
};

// 灯条建模 V4 (扩散模型)
class ErrorTerm {
public:
    ErrorTerm(const std::vector<double>& vals, int col, int row):
        values(vals), img_col(col), img_row(row){;}

    ~ErrorTerm(){;}

    /// @brief top: 上顶点 ctr: 中点 dec: 衰减函数
    template <typename T>
    bool operator() (const T* const _top, const T* const _ctr, const T* const _dec, T* residual) const {
        Eigen::Matrix<T, 2, 1> top(_top[0], _top[1]);
        Eigen::Matrix<T, 2, 1> ctr(_ctr[0], _ctr[1]);
        Eigen::Matrix<T, 2, 1> t2c = ctr - top;         // top -> center
        T t2c_norm = t2c.norm();                        // 法向量需要单位化
        Eigen::Matrix<T, 2, 1> bottom = ctr + t2c;
        Eigen::Matrix<T, 2, 1> normal(t2c(1) / t2c_norm, - t2c(0) / t2c_norm);
        std::vector<T> results(img_row * img_col);
        #pragma omp parallel for num_threads(8) 
        for (int i = 0; i < img_row; i++) {
            int base = i * img_col;
            for (int j = 0; j < img_col; j++) {
                Eigen::Matrix<T, 2, 1> t2n(T(j) - top(0), T(i) - top(1)), b2n(T(j) - bottom(0), T(i) - bottom(1));
                T t_len = t2n.norm(), b_len = b2n.norm(); 
                T prod = b_len >= t_len ? t2n.dot(t2c) : b2n.dot(- t2c);  // 更加靠近顶部 ? 内积求夹角cos : -t2c = b2c
                T dist = T(0);
                if (prod >= T(0)) {                        // 90度夹角之内
                    dist = b_len >= t_len ? ceres::abs(t2n.dot(normal)) : ceres::abs(b2n.dot(normal));
                }
                else {
                    dist = b_len >= t_len ? t_len : b_len;
                }
                T decay = T(1) / (ceres::exp(_dec[0] * (dist - _dec[1])) + T(1));     // 计算光线衰减
                int index = base + j;
                T value = T(values[index]);
                results[index] = ceres::abs(value - decay);                           // 简单L1 Loss
            }
        }
        residual[0] = std::accumulate(results.begin(), results.end(), T(0));
        return true;
    }

    static ceres::CostFunction* Create(const std::vector<double>& vals, int col, int row) {
        return new ceres::AutoDiffCostFunction<ErrorTerm, 1, 2, 2, 2>(
            new ErrorTerm(vals, col, row)
        );
    }
public:
    const std::vector<double>& values;
    int img_col;
    int img_row;
};

#endif  //__ERROR_TERM_HPP