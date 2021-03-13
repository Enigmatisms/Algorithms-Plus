#ifndef __ERROR_TERM_HPP
#define __ERROR_TERM_HPP
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <vector>
#include <utility>
#include <opencv2/core.hpp>

#define print_var(x) (std::cout << #x << ": " << x << std::endl)
using ptsType = std::vector<std::pair<double, cv::Point2d> >;
#define _A 2.5

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
    ErrorTerm(const std::vector<double>& vals, int col, int row, double radius):
        values(vals), img_col(col), img_row(row){
            _a = _A;
            _b = radius;
            cx = double(img_col) / 2;
            cy = double(img_row) / 2;
            printf("Img col: %d, img_row: %d, cx: %lf, cy: %lf\n", img_col, img_row, cx, cy);
        }

    ~ErrorTerm(){;}

    /// @brief top: 上顶点 ctr: 中点 dec: 衰减函数
    template <typename T>
    bool operator() (const T* const _top, const T* const _ctr, T* residual) const {
        Eigen::Matrix<T, 2, 1> top(_top[0], _top[1]);
        Eigen::Matrix<T, 2, 1> ctr(_ctr[0], _ctr[1]);
        Eigen::Matrix<T, 2, 1> t2c = ctr - top;         // top -> center
        T t2c_norm = t2c.norm();                        // 法向量需要单位化
        Eigen::Matrix<T, 2, 1> bottom = ctr + t2c;
        Eigen::Matrix<T, 2, 1> normal(t2c(1) / t2c_norm, - t2c(0) / t2c_norm);
        T b = T(_b) + _ctr[2];                           // ctr[2] 不是中心,是扩散半径
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
                T decay = T(1) / (ceres::exp(T(_a) * (dist - b)) + T(1));     // 计算光线衰减
                int index = base + j;
                T value = T(values[index]);
                results[index] = ceres::pow(value - decay, 2);
            }
        }

        /// 添加一些惩罚项
        residual[0] = std::accumulate(results.begin(), results.end(), T(0));
        T extra_loss(0.0);
        // centralPunish<T>(_ctr, extra_loss);
        borderPunish<T>(top(0), top(1), extra_loss);
        borderPunish<T>(bottom(0), bottom(1), extra_loss);
        std::cout << "B is:" << b << std::endl;
        std::cout << "bot is:" << bottom(0) << "," << bottom(1) << std::endl;
        rangePunish<T>(b, extra_loss);
        residual[0] += extra_loss;
        
        // printf("Residual %lf\n", residual[0]);
        // if (change_cost < 8.0) {            // 初始情况下,半径不做
        //     change_cost ++;
        // }
        // residual[0] += T(change_cost) * (ceres::pow(_ctr[2], 2));
        return true;
    }

    static ceres::CostFunction* Create(const std::vector<double>& vals, int col, int row, double radius) {
        return new ceres::AutoDiffCostFunction<ErrorTerm, 1, 2, 3>(
            new ErrorTerm(vals, col, row, radius)
        );
    }

private:
    /// ============== 惩罚化的弱约束问题 ==============
    /// 中心位置参数的惩罚
    template <typename T>
    inline void centralPunish(const T* const ctr, T& term) const {
        T factor = ceres::pow(ctr[0] - cx, 2) + ceres::pow(ctr[1] - cy, 2);
        if (factor > T(25.0)) {      // 说明超过了中心可以移动的范围
            term += factor;
        }
    }

    /// 边界惩罚
    template <typename T>
    inline void borderPunish(const T& x, const T& y, T& term) const {
        if (x < T(0.0)) {
            term += ceres::pow(x, 2);
        }
        else if (x > T(img_col)) {
            term += ceres::pow(x - T(img_col), 2);
        }
        if (y < T(0.0)) {
            term += ceres::pow(y, 2);
        }
        else if (y > T(img_row)) {
            term += ceres::pow(y - T(img_row), 2);
        }
    }

    /// 扩散范围惩罚
    template <typename T>
    inline void rangePunish(const T& b, T& term) const {
        if (b < T(1.1)) {
            term += T(9.0) * (T(1.1) - b);
        }
        else if (b > T(2.4)) {
            term += T(9.0) * (b - T(2.4));
        }
    }

public:
    static double change_cost;
    const std::vector<double>& values;
    int img_col;
    int img_row;

    double cx;
    double cy;
    double _a;
    double _b;
};

#endif  //__ERROR_TERM_HPP