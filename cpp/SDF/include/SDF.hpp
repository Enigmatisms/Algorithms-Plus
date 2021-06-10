#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <unordered_map>

/** 基于SDF的Mesh合并方法实现 */

typedef struct Edge {
    Eigen::Vector2d sp;
    Eigen::Vector2d ep;
    double weight;
};

typedef std::vector<Eigen::Vector3d> Mesh;
typedef Eigen::Matrix<uchar, -1, -1> MatrixXu;
typedef std::vector<Edge> Edges;

struct hashFunctor {
    template<typename T>
    uint64_t operator() (const std::pair<T, T>& pr) const {
        uint64_t h1 = std::hash<T>{}(pr.first);
        uint64_t h2 = std::hash<T>{}(pr.second);
        return h1 ^ h2;
    }
};

struct equalFunctor {
    template<typename T>
    bool operator() (const std::pair<T, T>& p1, const std::pair<T, T>& p2) const {
        return (p1.first == p2.first) && (p1.second == p2.second);
    }
};

typedef std::unordered_map<std::pair<int, int>, Edges, hashFunctor, equalFunctor> EdgeMap;


const double grid_size = 0.1;           // Marching Square使用的grid大小

class SDF {
using Vertex = std::vector<std::pair<uchar, uchar>>;
public:
    SDF() {}
    ~SDF() {}
public:
    /**
     * @brief 将两个待匹配的Mesh限制在一个有裕量的BoundingBox里面
     * @param m1m2 是两个Mesh （为了方便起见，只考虑两个mesh具有重合的部分）
     * @param t1br boundingBox的左上和右下点, tl = (w, x), br = (y, z) 
     * @note 对于实际的两个mesh（能分段）也比较容易实现，只需要划分grid就好
     */
    void meshBoundingBox(const Edges& m1, const Edges& m2, Eigen::Vector2d& tl, Eigen::Vector2i& rnc) const;
private:
    void singleMeshSDF(const Edges& m, const Eigen::Vector2d& tl, const Eigen::Vector2i& grid_size,
        Eigen::MatrixXd& sdf, Eigen::MatrixXd& alpha, Eigen::MatrixXi& idx) const;
    /**
     * @brief 计算出来的应该是 每个Grid上的SDF值，这样会方便之后的Marching Squares计算
     */
    void doubleMeshSDF(const Edges& m1, const Edges& m2, Eigen::MatrixXd& sdf) const ;


    /// @todo
    enum calcType {
        START   = 0,
        NORMAL  = 1,
        END     = 2,
    };
    template<calcType type>
    void singleMeshPieceSDF(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& sdf) const;

    /// @todo
    /// @brief 计算首尾段的val 直接修改结果alpha
    void alphaCalculation(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& alpha, bool truc_start) const;

    /// @todo
    /**
     * @brief 计算不同Grid对应的tags 以更快地确定如何进行交点计算
     */
    void gridTagsCalculation(const Eigen::MatrixXd& sdf, MatrixXu& tags) const;

    /**
     * @brief Marching Cubes的2D算法，Marching Squares
     * @note 首先需要根据SDF求出每个Grid的值，根据对应Grid选择等值线线性插值计算的位置（得到了一个个点）
     * @note 计算完插值位置之后需要顺序化
     * @note 求出的所有边先放在一个unordered_map里面，之后按照顺序组合
     */
    void marchingSquare(const Eigen::MatrixXd& sdf1, const Eigen::MatrixXd& sdf2, const Eigen::Vector4d& tlbr, EdgeMap& dst) const;

private:
    // 输入
    /// @todo
    void linearInterp(const Eigen::Vector4d& vals, const Vertex& pr, bool neg, Edges& dst) const;
private:
    /**
     * 点顺序定义为： 这也是vals的(w, x, y, z) 注意激光扫描的方向是逆时针 此部分为线性插值
     * p0   边0  p1
     * 边3       边1
     * p3   边2  p2
     */ 
    // 使用8种情况表示 不同等高线情况，直接取p0恒为正数，对于不同的情况只需调换顺序
    std::array<Vertex, 8> tb = {
        Vertex{},
        Vertex{{2, 3}},
        Vertex{{1, 2}},
        Vertex{{1, 3}},

        Vertex{{0, 1}},
        Vertex{{0, 3}, {1, 2}},
        Vertex{{0, 2}},
        Vertex{{0, 3}},
    };
};