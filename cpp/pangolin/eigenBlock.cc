#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

template<typename T, int rows, int cols>
void printMat(const Eigen::Matrix<T, rows, cols>& M){
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            std::cout << M(i, j) << ", ";
        }
        std::cout << std::endl;
    }
}

int main(){
    Eigen::Matrix<double, 3, 6> vex;
    vex << 1, 2, 3, 4, 5, 6,
        6, 5, 4, 3, 2, 1,
        0, 0, 0, 0, 0, 0;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.5;
    std::cout << "Before process: \n";
    printMat<double, 3, 6>(vex);
    for (int i = 0; i < 6; i++){
        vex.block<3, 1>(0, i) = R * vex.block<3, 1>(0, i);
    }
    std::cout << "After process: \n";
    printMat<double, 3, 6>(vex);
    return 0;
}