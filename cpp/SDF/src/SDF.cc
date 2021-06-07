#include "../include/SDF.hpp"

void SDF::meshBoundingBox(const Mesh& m1, const Mesh& m2, Eigen::Vector4d& tlbr) const {

}

void SDF::singleMeshSDF(const Mesh& mesh, const Eigen::Vector4d& tlbr, Eigen::MatrixXd& sdf) const {

}

void SDF::marchingSquare(const Eigen::MatrixXd& sdf1, const Eigen::MatrixXd& sdf2, const Eigen::Vector4d& tlbr, Mesh& dst) const {
    Eigen::MatrixXd lut = sdf1 + sdf2;          // SDF look up table
    int cols = lut.cols(), rows = lut.rows();
    double sx = tlbr.w(), sy = tlbr.x(), ex = tlbr.y(), ey = tlbr.z();
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            
        }
    }
}