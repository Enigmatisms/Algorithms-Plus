#include <iostream>
#include <ceres/ceres.h>

using namespace std;
using namespace ceres;

struct CostFunc
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = pow(T(10.0) - x[0], 2);
        return true;
    }
};

int main(int argc, char* argv[]) {
    double initial_x = 5.0;
    double x = initial_x;

    Problem problem;
    CostFunction* cost_func = new AutoDiffCostFunction<CostFunc, 1, 1>(new CostFunc);
    problem.AddResidualBlock(cost_func, NULL, &x);

    Solver::Options options;
    options.linear_solver_type = DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;
    cout << "x: " << initial_x << " -> " << x << endl;
    return 0;
}