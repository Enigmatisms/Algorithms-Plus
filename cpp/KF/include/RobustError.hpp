#ifndef __ROBUST_STATE_HPP
#define __ROBUST_STATE_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 12, 12> Mat12d;
typedef Eigen::Matrix<double, 12, 1> Vec12d;
typedef Eigen::Matrix<double, 12, 6> HalfMat12d;
typedef Eigen::Matrix<double, 1, 6> RowVector6d;

/// 状态后验的鲁棒估计
class RobustStateProb {
public:
    /// Huber函数的参数可以在外部设置
    RobustStateProb(const HalfMat12d& _X, const Vec12d& _y, double dt):
        X(_X), y(_y)
    {
        delta = dt;
    }

    ~RobustStateProb() {}

    static ceres::CostFunction* Create(const HalfMat12d& _X, const Vec12d& _y, double dt) {
        return new ceres::AutoDiffCostFunction<RobustStateProb, 1, 6>(
            new RobustStateProb(_X, _y, dt)
        );
    }

    template <typename T>
    bool operator() (const T* const beta, T* residual) const {
        residual[0] = T(0);
        Eigen::Matrix<T, 6, 1> Beta;
        Beta << beta[0], beta[1], beta[2], beta[3], beta[4], beta[5];
        for (int i = 0; i < 12; i++) {
            Eigen::Matrix<T, 1, 6> Xt;
            Xt << T(X(i, 0)), T(X(i, 1)), T(X(i, 2)), T(X(i, 3)), T(X(i, 4)), T(X(i, 5));
            T res = T(y(i)) - Xt.dot(Beta);
            if (res >= T(delta)) {
                residual[0] += T(2) * T(delta) * (res - T(delta));
            }
            else if (res <= T(-delta)) {
                residual[0] += T(2) * T(delta) * (-res - T(delta));
            }
            else {
                residual[0] += ceres::pow(res, 2);
            }
        }
    }
private:
    const HalfMat12d& X;
    const Vec12d& y;
    double delta;
};

/// 观测噪声协方差矩阵（的方差）的鲁棒估计
class RobustCovProb {
    ;
};

#endif  //__ROBUST_STATE_HPP