#pragma once
/// @brief mergeMesh SDF types
#include <Eigen/Core>
#include <Eigen/Dense>
#include <queue>
#include <vector>
#include <unordered_map>
#include <opencv2/core.hpp>

struct Edge {
    Eigen::Vector2d sp;
    Eigen::Vector2d ep;
    double weight;
    Edge() {}
    Edge(const Eigen::Vector2d& _sp, const Eigen::Vector2d& _ep): sp(_sp), ep(_ep), weight(1.0) {}    

    bool isAngleAcute(const Edge& eg) const {       // 两个edge夹角是否是锐角
        return ((ep - sp).dot(eg.ep - eg.sp) > 0.0);
    }

    bool sameDirectionAsNormal(const Edge& eg) const {
        Eigen::Vector2d vec = eg.ep - eg.sp;
        Eigen::Vector2d normal = Eigen::Vector2d(-vec.y(), vec.x());
        return (normal.dot(ep - sp) > 0);
    }

    void swap() {
        std::swap(sp, ep);
    }
};

typedef std::vector<Eigen::Vector2d> Mesh;
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
typedef std::unordered_map<std::pair<int, int>, bool, hashFunctor, equalFunctor> EdgeBool;
typedef std::deque<std::pair<int, int> > IntPrs;

extern const cv::Vec3b color_r;
extern const cv::Vec3b color_y;
extern const cv::Vec3b color_g;
extern const cv::Vec3b color_b;
extern const cv::Vec3b color_w;
extern const cv::Vec3b color_k;