#include "include/NDT.hpp"

int main(int argc, char* argv[]){
    double _x = (double)atof(argv[1]);
    double _y = (double)atof(argv[2]);
    Eigen::Vector2d mu(5, 5);
    Eigen::Vector2d x(_x, _y);
    Eigen::Matrix2d icov;
    icov << 0.23, 0.12, 0.12, 0.48;
    Eigen::Vector2d vec = x - mu;
    auto product = vec.transpose() * icov * vec;
    std::cout << "Product: " << product.value() << std::endl;
    return 0;
}
