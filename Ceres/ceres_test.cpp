#include <iostream>
#include <ceres/ceres.h>

using namespace std;
using namespace ceres;

struct CostFunc
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(-6.81255495e-13) * pow(x[0], 4) + T(2.23967582e-09) * pow(x[0], 3) +
            T(-2.61811669e-06) * pow(x[0], 2) + T(1.26270659e-03) * x[0] + 
            T(-2.02581731e-01) + (T(-0.000010) * x[0] * x[0] + T(-0.711466)) / 
                pow((T(0.000010) * x[0] * x[0] - T(0.711466)), 2) / T(1382.085566405469);
        return true;
    }
};

int main(int argc, char* argv[]) {
    double initial_x = 780;
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