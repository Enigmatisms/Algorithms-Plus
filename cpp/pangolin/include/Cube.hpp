#ifndef __CUBE_HPP__
#define __CUBE_HPP__

// 不使用ROS进行转递

#include <memory>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

typedef int Axis;
typedef bool Dir;
typedef unsigned char uchar;

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define FORWARD true
#define BACKWARD false

/// 4个顶点的索引 + step
const int edges[3][5] = {
    {0, 1, 4, 5, 2},
    {0, 1, 2, 3, 4},
    {0, 2, 4, 6, 1}
};

class Cube{
public:
    Cube(double x, double y, double z, double l){
        double f = l / 2;
        ptr = new double[72];
        vertices << x-f, y-f, z-f, x-f, y-f, z+f, x+f, y-f, z-f, x+f, y-f, z+f,
            x-f, y+f, z-f, x-f, y+f, z+f, x+f, y+f, z-f, x+f, y+f, z+f;
        this->l = l;
        initial << x, y, z;
        vel << 0.02, 0.02, 0.01;
        w = 0.02;
    }
    ~Cube(){
        delete [] ptr;
    }
public:
    /// 旋转的轴与方向（FORWARD表示右手螺旋，绕轴逆时针旋转）
    void reset();
    void rotate(Axis axis = AXIS_Z, Dir direct = FORWARD);          // 旋转
    void move(Axis axis = AXIS_X, Dir direct = FORWARD);            // 平移
    double* getCurrentFrame();                    // 用于绘制线框，生成点的位置关系
private:
    Eigen::Matrix<double, 3, 8> vertices;
    Eigen::Vector3d initial;
    Eigen::Vector3d vel;
    double* ptr;
    double l;
    double w;
};

void Cube::rotate(Axis axis, Dir direct){
    double _w = (direct == FORWARD) ? w : - w;
    Eigen::Vector3d _ax = Eigen::Vector3d::Zero();
    _ax(axis) = 1.0;
    Eigen::AngleAxisd rot(_w, _ax);
    Eigen::Matrix3d R = rot.toRotationMatrix();
    for (int i = 0; i < 8; i++){
        vertices.block<3, 1>(0, i) = R * vertices.block<3, 1>(0, i);
    }
}

void Cube::reset(){
    double f = l/2, x = initial(0), y = initial(1), z = initial(2);
    vertices << x-f, y-f, z-f, x-f, y-f, z+f, x+f, y-f, z-f, x+f, y-f, z+f,
            x-f, y+f, z-f, x-f, y+f, z+f, x+f, y+f, z-f, x+f, y+f, z+f;
}

double* Cube::getCurrentFrame(){
    for (int d = 0; d < 3; d++){
        int step = edges[d][4];
        for (int v = 0; v < 4; v++){
            const Eigen::Vector3d& dfrom = vertices.col(edges[d][v]);
            const Eigen::Vector3d& dto = vertices.col(edges[d][v + step]);
            for (int i = 0; i < 3; i++){
                ptr[24 * d + 6 * v + i] = dfrom(i);
            }
            for (int i = 0; i < 3; i++){
                ptr[24 * d + 6 * v + i + 3] = dto(i);
            }
        }
    }
    return ptr;
}

void Cube::move(Axis axis, Dir direct){
    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    trans(axis) = vel(axis);
    if (direct == BACKWARD){
        trans *= -1;
    }
    for (int i = 0; i < 8; i++){
        vertices.block<3, 1>(0, i) += trans;
    }
}


#endif  //__CUBE_HPP__