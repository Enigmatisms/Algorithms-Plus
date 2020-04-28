#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

Eigen::Matrix4i A;

int main(int, char **){
    A << 0, 1, 0, 1,
         0, 0, 1, 1,
         0, 1, 0, 1,
         0, 1, 0, 0;
    Eigen::Matrix<int, 4, 4> AT = A.transpose();
    Eigen::Matrix4i out = A*AT;
    Eigen::Matrix4i A2 = A * A;
    Eigen::Matrix4i A3 = A2 * A;
    Eigen::Matrix4i A4 = A3 * A;

    std::cout << A << std::endl << std::endl;
    std::cout << AT << std::endl << std::endl;
    std::cout << out << std::endl << std::endl;
    std::cout << A2 << std::endl << std::endl;
    std::cout << A3 << std::endl << std::endl;
    std::cout << A4 << std::endl << std::endl;
    return 0;
}