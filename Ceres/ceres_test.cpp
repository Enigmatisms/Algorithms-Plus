#include <iostream>
#include <ceres/ceres.h>

using namespace std;
using namespace ceres;

struct CostFunc
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(2.61915134e-13) * pow(x[0], 5) + T(-9.74186967e-10) * pow(x[0], 4) +
            T(1.40707957e-06) * pow(x[0], 3) + T(-9.79470420e-04) * x[0] * x[0] + 
            T(3.25042702e-01) * x[0] + T(-4.03976079e+01) +
                (T(-0.000010) * x[0] * x[0] + T(-0.711466)) / 
                pow((T(0.000010) * x[0] * x[0] - T(0.711466)), 2) / T(1382.085566405469);
        return true;
    }
};

int main(int argc, char* argv[]) {
    double initial_x = 560;
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