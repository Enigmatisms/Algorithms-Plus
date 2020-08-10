#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include "Record.hpp"

using namespace std;

int main(int, char **){
    cv::RNG rng;
    Record<cv::Point2d> rcd;
    for(double x = 0; x < 9.0; x+= 0.2){
        double y = 2 * x * x - 8 * x + 8 + rng.uniform(-1.0 - x / 10, 1.0 + x / 10)
            + rng.gaussian(1 + x / 4);        
        rcd.input(x, y);
    }
    rcd.writeBinInFile("/home/sentinel/Algorithms-Plus/Ceres/bin.txt");
    rcd.writeStrInFile("/home/sentinel/Algorithms-Plus/Ceres/pydata.txt");

    printf("Random generation and output completed.\n");
    return 0;
}

