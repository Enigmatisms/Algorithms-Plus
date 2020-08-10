#include <ctime>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include "Record.hpp"

// 尝试拟合一个二次函数?

class CostFunctor{
public:
    CostFunctor(double x, double y){
        _x = x;
        _y = y;
    }

    template <typename T>
    bool operator()(const T* const a, T* residual) const{
        T x = T(_x);
        T y = T(_y);
        residual[0] = pow(y - a[0] * x * x - a[1] * x - a[2], 2);
        return true;
    }
public:
    double _x;
    double _y;
};

int main(int, char **){
    Record<cv::Point2d> rcd;
    rcd.readFromFile("/home/sentinel/Algorithms-Plus/Ceres/bin.txt");
    double init_a[3] = {1, 3, 4};
    ceres::Problem prob;
    for(const cv::Point2d &pt: rcd.storage){
        ceres::CostFunction* cost_func = 
            new ceres::AutoDiffCostFunction<CostFunctor, 1, 3>
            (new CostFunctor(pt.x, pt.y));
        prob.AddResidualBlock(cost_func, nullptr, init_a);
    }

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.minimizer_progress_to_stdout = false;
    opts.max_linear_solver_iterations = 2000;
    opts.function_tolerance = 10e-12;
    ceres::Solver::Summary summary;

    ceres::Solve(opts, &prob, &summary);

    //std::cout << summary.FullReport() << std::endl;
    printf("%lf %lf %lf\n", init_a[0], init_a[1], init_a[2]);
    return 0;

    return 0;
}