#include "include/SDF.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./march <threshold>";
        return -1;
    }
    double threshold = atof(argv[1]);
    SDF sdf(threshold);
    Mesh mesh = {
        Eigen::Vector2d(60, 120),
        Eigen::Vector2d(600, 180),
        Eigen::Vector2d(700, 280),
        Eigen::Vector2d(750, 500),
        Eigen::Vector2d(730, 600),
        Eigen::Vector2d(500, 650),
        Eigen::Vector2d(400, 700),
        Eigen::Vector2d(300, 300),
        Eigen::Vector2d(200, 600),
        Eigen::Vector2d(100, 200),
    }, noised_mesh;
    Mesh mesh2 = {
        Eigen::Vector2d(300, 300),
        Eigen::Vector2d(200, 600),
        Eigen::Vector2d(100, 200),
    };

    Edges edges, noised_edges, results;
    sdf.addNoise2ExistingMesh(mesh, noised_mesh, 5);
    sdf.mesh2Edges(mesh, edges);
    sdf.mesh2Edges(noised_mesh, noised_edges);
    Eigen::MatrixXd sdf_res, alpha, noised_res, noised_alpha;
    Eigen::MatrixXi belongs, noised;
    sdf.mergeMesh(edges, noised_edges, 1.0, results);
    return 0;
}