#include "include/SDF.hpp"

int main() {
    SDF sdf;
    cv::Mat image(800, 1000, CV_8UC3);
    cv::rectangle(image, cv::Rect(0, 0, 1000, 800), cv::Vec3b(0, 0, 0), -1);

    Mesh mesh = {
        Eigen::Vector2d(60, 120),
        Eigen::Vector2d(180, 160),
        Eigen::Vector2d(290, 240),
        Eigen::Vector2d(400, 160),
        Eigen::Vector2d(500, 500),
        Eigen::Vector2d(600, 400),
        Eigen::Vector2d(700, 600),
        Eigen::Vector2d(750, 620),
        Eigen::Vector2d(850, 670),
        Eigen::Vector2d(950, 680),
    }, noised_mesh;
    Edges edges, noised_edges;
    sdf.addNoise2ExistingMesh(mesh, noised_mesh, 50);
    sdf.mesh2Edges(mesh, edges);
    sdf.mesh2Edges(noised_mesh, noised_edges);
    Eigen::MatrixXd sdf_res, alpha, noised_res, noised_alpha;
    Eigen::MatrixXi belongs, noised;
    sdf.doubleMeshSDF(edges, noised_edges, sdf_res);
    return 0;
}