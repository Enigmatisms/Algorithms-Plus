#ifndef __ROBUST_STATE_HPP
#define __ROBUST_STATE_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <deque>
#include <vector>

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
        T b = T(delta);
        for (int i = 0; i < 12; i++) {
            Eigen::Matrix<T, 1, 6> Xt;
            Xt << T(X(i, 0)), T(X(i, 1)), T(X(i, 2)), T(X(i, 3)), T(X(i, 4)), T(X(i, 5));
            T res = T(y(i)) - Xt.dot(Beta);
            if (res >= b) {
                residual[0] += T(2) * b * (res) - b;
            }
            else if (res <= -b) {
                residual[0] += T(2) * b * (-res) - b;
            }
            else {
                residual[0] += ceres::pow(res, 2);
            }
        }
        return true;
    }
private:
    const HalfMat12d& X;
    const Vec12d& y;
    double delta;
};

/// 观测噪声协方差矩阵（的方差）的鲁棒估计
class RobustCovProb {
public:
    RobustCovProb(const std::deque<double>& _rs, double _r_med, double _delta):
        rs(_rs), r_med(_r_med), delta(_delta)
    {
        ;
    }
    
    ~RobustCovProb(){}

    static double MakeProb2Solve(const std::deque<double>& _rs, double _r_med, double dt, double* res) {
        ceres::Problem prob;
        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<RobustCovProb, 1, 1>(
            new RobustCovProb(_rs, _r_med, dt)
        );
        prob.AddResidualBlock(cost, nullptr, res);
        ceres::Solver::Options opts;
        opts.linear_solver_type = ceres::DENSE_QR;
        opts.minimizer_type = ceres::LINE_SEARCH;
        opts.line_search_direction_type = ceres::LBFGS;
        opts.minimizer_progress_to_stdout = false;
        opts.max_linear_solver_iterations = 50;
        opts.function_tolerance = 1e-6;
        ceres::Solver::Summary summary;
        ceres::Solve(opts, &prob, &summary);
    }

    template <typename T>
    bool operator() (const T* const r_hat, T* residual) const {
        residual[0] = T(0);
        const T b = T(delta);
        for (double value: rs) {
            T val = (T(value) - r_hat[0]) / T(r_med);
            if (val >= b) {
                residual[0] += T(2) * b * (val) - b;
            }
            else if (val <= -b) {
                residual[0] += T(2) * b * (-val) - b;
            }
            else {
                residual[0] += ceres::pow(val, 2);
            }
        }
        // residual[0] += T(0.01) * ceres::pow(r_hat[0], 2);
        return true;
    }
private:
    const std::deque<double>& rs;
    const double r_med;
    const double delta;
};

#endif  //__ROBUST_STATE_HPP