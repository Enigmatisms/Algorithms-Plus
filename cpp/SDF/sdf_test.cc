#include "include/SDF.hpp"

int main() {
    SDF sdf;
    cv::Mat image(800, 1000, CV_8UC3);
    cv::Mat img2(800, 1000, CV_8UC3);
    cv::rectangle(image, cv::Rect(0, 0, 1000, 800), cv::Vec3b(0, 0, 0), -1);
    cv::rectangle(img2, cv::Rect(0, 0, 1000, 800), cv::Vec3b(0, 0, 0), -1);
    // Mesh mesh = {
    //     Eigen::Vector2d(60, 120),
    //     Eigen::Vector2d(600, 180),
    //     Eigen::Vector2d(700, 280),
    //     Eigen::Vector2d(750, 500),
    //     Eigen::Vector2d(730, 600),
    //     Eigen::Vector2d(500, 650),
    //     Eigen::Vector2d(400, 700),
    //     Eigen::Vector2d(300, 300),
    //     Eigen::Vector2d(200, 600),
    //     Eigen::Vector2d(100, 200),
    // }, noised_mesh;

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
    sdf.singleMeshSDF(edges, Eigen::Vector2d(0, 0), Eigen::Vector2i(800, 1000), sdf_res, alpha, belongs);
    sdf.singleMeshSDF(noised_edges, Eigen::Vector2d(0, 0), Eigen::Vector2i(800, 1000), noised_res, noised_alpha, noised);
    sdf.visualizeValues(sdf_res, Eigen::Vector2d(0, 0), image);
    sdf.visualizeValues(noised_res, Eigen::Vector2d(0, 0), img2);
    sdf.visualizeMesh(mesh, color_g, image);
    sdf.visualizeEdges(noised_edges, color_k, img2);
    cv::imshow("disp", image);
    cv::imshow("noised", img2);
    cv::waitKey(0);
    return 0;
}