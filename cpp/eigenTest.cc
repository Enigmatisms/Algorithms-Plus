#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

Eigen::Matrix4i A;

template<typename Ty = double, int x = 4, int y = 4>
Eigen::Matrix<Ty, x, y> getMatrix(){
    using Matrix = Eigen::Matrix<Ty, x, y>;
    Matrix mat = Matrix::Identity();
    return mat;
}

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

    Eigen::Matrix4d test = getMatrix();
    std::cout << "Test:" << std::endl;
    std::cout << test << std::endl << std::endl;

    Eigen::Matrix<double, 4, 4> test2;
    Eigen::Matrix<double, 4, 1> test3;

    std::cout << "Test2:" << std::endl;
    std::cout << test2 << std::endl << std::endl;
    test2.setIdentity();
    test2 *= 2;
    std::cout << "Test2:" << std::endl;
    std::cout << test2 << std::endl << std::endl;
    test2 = test2.inverse();
    std::cout << "Inversed test2:" << std::endl;
    std::cout << test2 << std::endl << std::endl;


    test3.setZero();
    std::cout << "Test3:" << std::endl;
    std::cout << test3 << std::endl << std::endl;

    Eigen::Matrix<double, 1, 4> test4 = test3.transpose();
    std::cout << "Test4:" << std::endl;
    std::cout << test4 << std::endl << std::endl;

    Eigen::Matrix3d mat; 
    mat << 4, 2, 3,
        5, 1, 5, 
        6, 3, 2;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << "U:" << std::endl;
    std::cout << svd.matrixU() << std::endl;
    std::cout << "V:" << std::endl;
    std::cout << svd.matrixV() << std::endl;
    std::cout << "Singular vector:" << std::endl;
    std::cout << svd.singularValues() << std::endl;
    std::cout << "Singular matrix:" << std::endl;
    std::cout << svd.singularValues().asDiagonal().toDenseMatrix() << std::endl << std::endl;
    
    std::cout << A << std::endl << std::endl;
    std::cout << AT << std::endl << std::endl;
    std::cout << out << std::endl << std::endl;
    std::cout << A2 << std::endl << std::endl;
    std::cout << A3 << std::endl << std::endl;
    std::cout << A4 << std::endl << std::endl;
    return 0;
}