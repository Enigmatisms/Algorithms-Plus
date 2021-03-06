#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <stack>
#include <queue>
#include "meshTypes.h"

/** 基于SDF的Mesh合并方法实现
 * @todo  1. 顺序化的实现不太好
 * @todo  2. 没有邻近 mesh 查找，需要提供两个已经匹配的mesh，并给权重赋值
 * @todo  3. 可能有锯齿形的边
 * @todo  4. 使用alpha进行长度截断 效果判定
*/
#define DEBUG

class SDF {
using Vertex = std::vector<std::pair<uchar, uchar>>;
public:
    SDF(double th): threshold(th), grid_size(0.1) {}
    ~SDF() {}
public:
    /// @brief 接口：输入两个Edges（定义见头文件），输出一个Edges (实际上不应该叫Edge，应该叫Arc)
    /// @param eg1eg2 两个封装好带权重的mesh
    /// @param _gsize 设置的grid大小，单位m
    /// @param dst 输出的Edges
    void mergeMesh(const Edges& eg1, const Edges& eg2, double _gsize, Edges& dst);

    /// @brief Mesh 转换为 Edges 这个不建议使用，除非edge不需要设置权重（初始化为1） 
    void mesh2Edges(const Mesh& mesh, Edges& edges) const {
        for (size_t i = 0; i < mesh.size() - 1; i++)
            edges.emplace_back(mesh[i], mesh[i+1]);
    }

    /// @brief 转换 Edges 为 Mesh
    void edges2Mesh(const Edges& edges, Mesh& mesh) const {
        for (size_t i = 0; i < edges.size() - 1; i++)
            mesh.push_back(edges[i].sp);
        mesh.push_back(edges.back().sp);
        mesh.push_back(edges.back().ep);
    }
private:
    /**
     * @brief 将两个待匹配的Mesh限制在一个有裕量的BoundingBox里面
     * @param m1m2 是两个Mesh （为了方便起见，只考虑两个mesh具有重合的部分）
     * @param tl boundingBox的左上点
     * @param rnc row and col
     * @note 对于实际的两个mesh（能分段）也比较容易实现，只需要划分grid就好 裕量为两个grid的大小
     */
    void meshBoundingBox(const Edges& m1, const Edges& m2, Eigen::Vector2d& tl, Eigen::Vector2i& rnc) const;
    /// @brief 输入一个完整mesh + boundingBox左上角anchor + boundingBox size，输出三通道：sdf / alpha / 最小值从属关系
    void singleMeshSDF(const Edges& m, const Eigen::Vector2d& tl, const Eigen::Vector2i& grid_size,
        Eigen::MatrixXd& sdf, Eigen::MatrixXd& alpha, Eigen::MatrixXi& idx) const;
    
    /**
     * @brief 计算单个mesh段产生的sdf，需要分情况：是否是起始mesh / 是否是末尾mesh，是否是中间段mesh
     * @note 假如是首尾两者之一，会附加alpha的计算，比中间段mesh更加复杂
     * @note 输入一个mesh片段，boundingBox左上角的位置tl，输出一个sdf，一个alpha
     */
    enum calcType {
        START   = 0,
        NORMAL  = 1,
        END     = 2,
    };
    template<calcType type>
    void singleMeshPieceSDF(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& sdf, Eigen::MatrixXd& alpha) const;

    /// @brief 计算首尾段的val 直接修改结果alpha
    void alphaCalculation(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& alpha, bool truc_start) const;

    /// @brief 计算不同Grid对应的tags 以更快地确定如何进行交点计算
    void gridTagsCalculation(const Eigen::MatrixXd& sdf, MatrixXu& tags) const;

    /**
     * @brief Marching Cubes的2D算法，Marching Squares
     * @note 首先需要根据SDF求出每个Grid的值，根据对应Grid选择等值线线性插值计算的位置（得到了一个个点）
     * @note 计算完插值位置之后需要顺序化
     * @note 求出的所有边先放在一个unordered_map里面，之后按照顺序组合
     */
    void marchingSquare(const Eigen::MatrixXd& lut, const Eigen::MatrixXd& weights, EdgeMap& dst) const;

    void searchSerialize(const Eigen::Vector2d& tl, const Eigen::MatrixXd& alpha, EdgeMap& emap, Edges& result) const;

    void linearInterp(const Vertex& vtx, const Eigen::Vector4d& wghts, const Eigen::Vector4d& vals, Edges& edges) const;
public:
    #ifdef DEBUG
    void addNoise2ExistingMesh(const Mesh& src, Mesh& dst, double sig) const {
        for (const Eigen::Vector2d& pt: src) {
            dst.push_back(pt + Eigen::Vector2d::Random() * sig);
        }
    }
    // =====================  DEBUG ========================
    void visualizeValues(const Eigen::MatrixXd& vals, const Eigen::Vector2d& tl, cv::Mat& dst) const;
    void visualizeMesh(const Mesh& mesh, const cv::Vec3b& color, cv::Mat& dst) const;
    void visualizeEdges(const Edges& edges, const cv::Vec3b& color, cv::Mat& dst, bool output = false) const;
    void visualizeMarchingSquare(const EdgeMap& emap, const Eigen::Vector2d& tl, const cv::Vec3b& color, cv::Mat& dst) const;
    void visualizeAlpha(const Eigen::MatrixXd& alpha, const Eigen::Vector2d& tl, cv::Mat& dst) const;
    void visualizeCombinedAlpha(const Eigen::MatrixXd& a1, const Eigen::MatrixXd& a2, const Eigen::Vector2d& tl, cv::Mat& dst) const;
    void visualizeBelongs(const Eigen::MatrixXi& belong, const Eigen::Vector2d& tl, cv::Mat& dst) const;
    #endif
private:
    /**
     * p3   边2  p2
     * 边3       边1
     * p0   边0  p1
     */ 
    // 使用8种情况表示 不同等高线情况，直接取p0恒为正数，对于不同的情况只需调换顺序
    std::array<Vertex, 16> tb = {
        Vertex{}, Vertex{{0, 3}}, Vertex{{1, 0}}, Vertex{{1, 3}},
        Vertex{{1, 2}}, Vertex{{2, 1}, {0, 3}}, Vertex{{2, 0}}, Vertex{{2, 3}},
        Vertex{{3, 2}}, Vertex{{0, 2}}, Vertex{{3, 2}, {1, 0}}, Vertex{{1, 2}},
        Vertex{{3, 1}}, Vertex{{0, 1}}, Vertex{{3, 0}}, Vertex{}
    };
    
    std::array<Eigen::Vector2d, 4> vertices = {
        Eigen::Vector2d(0, 1), Eigen::Vector2d(1, 1), Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 0)
    };

    std::array<std::pair<int, int>, 4> neighbor_4 = {
        std::pair<int, int>({1, 0}), std::pair<int, int>({-1, 0}), std::pair<int, int>({0, 1}), std::pair<int, int>({0, -1})
    };
    
    double threshold;
    double grid_size;           // Marching Square使用的grid大小
};