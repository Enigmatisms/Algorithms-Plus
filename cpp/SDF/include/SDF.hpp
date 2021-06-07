#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <boost/function.hpp>

/** 基于SDF的Mesh合并方法实现 */

typedef std::vector<Eigen::Vector2d> Mesh;
typedef Eigen::Matrix<uchar, -1, -1> MatrixXu;

const double grid_size = 0.1;           // Marching Square使用的grid大小

class SDF {
public:
    SDF() {}
    ~SDF() {}
public:
    /**
     * @brief 将两个待匹配的Mesh限制在一个有裕量的BoundingBox里面
     * @param m1m2 是两个Mesh （为了方便起见，只考虑两个mesh具有重合的部分）
     * @param t1br boundingBox的左上和右下点, tl = (w, x), br = (y, z) 
     * @note 对于实际的两个mesh（可能分段）也比较容易实现，只需要划分grid就好
     */
    void meshBoundingBox(const Mesh& m1, const Mesh& m2, Eigen::Vector4d& tlbr) const;

private:
    /**
     * @brief 计算出来的应该是 每个Grid上的SDF值，这样会方便之后的Marching Squares计算
     * 
     */
    void singleMeshSDF(const Mesh& mesh, const Eigen::Vector4d& tlbr, Eigen::MatrixXd& sdf) const ;

    /**
     * @brief 计算不同Grid对应的tags 以更快地确定如何进行交点计算
     */
    void gridTagsCalculation(const Eigen::MatrixXd& sdf, MatrixXu& tags);

    /**
     * @brief Marching Cubes的2D算法，Marching Squares
     * @note 首先需要根据SDF求出每个Grid的值，根据对应Grid选择等值线线性插值计算的位置（得到了一个个点）
     * @note 计算完插值位置之后需要顺序化
     */
    void marchingSquare(const Eigen::MatrixXd& sdf1, const Eigen::MatrixXd& sdf2, const Eigen::Vector4d& tlbr, Mesh& dst) const;
private:
    /**
     * 点顺序定义为： 这也是vals的(w, x, y, z) 注意激光扫描的方向是逆时针 此部分为线性插值
     * p0->p1
     * ^    v
     * p3<-p2
     */
    static void intersectCase_0(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {}

    static void intersectCase_1(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {
        sp(0) = vals(3) / (std::abs(vals(2)) + std::abs(vals(3)) + 1e-5);
        sp(1) = grid_size;
        ep(0) = 0.0;
        ep(1) = vals(3) / (std::abs(vals(3)) + std::abs(vals(0)) + 1e-5);
    }

    static void intersectCase_2(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_3(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_4(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_5(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_6(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_7(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_8(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_9(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_10(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_11(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }


    static void intersectCase_12(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_13(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_14(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }

    static void intersectCase_15(const Eigen::Vector4d& vals, Eigen::Vector2d& sp, Eigen::Vector2d& ep) {

    }
};