#include "BA/bundleAdjustment.hpp"
#include "BA/_common.hpp"
#include <vector>

std::string path = "/home/sentinel/testPics/";

int main(){
    BALProblem bal(path + "problem-16-22106-pre.txt");
    bal.Perturb(1.0, 1.0, 1.0);
    ceres::Problem prob;
    const double* const points = bal.observations();

    std::cout << ">>> Data reading process is completed.\n";

    for (int i = 0; i < bal.num_observations(); i++){
        ceres::CostFunction* res_block = BAErrorTerm::Create(points[2 * i], points[2 * i + 1]);
        double* params = bal.mutable_camera_for_observation(i);
        double* point = bal.mutable_point_for_observation(i);
        ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.0);
        prob.AddResidualBlock(res_block, huber_loss, params, point);
    }

    std::cout << ">>> Residual blocks are set up.\n";

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;          // 舒尔补
    opts.minimizer_progress_to_stdout = true;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;

    std::cout << ">>> Prepare to solve...\n";
    ceres::Solve(opts, &prob, &summary);
    std::cout << summary.FullReport() << std::endl;
    bal.WriteToFile(path + "optimized.txt");
    bal.WriteToPLYFile(path + "optimized.ply");
    std::cout << "Output completed.\n" << std::endl;
    return 0;
}