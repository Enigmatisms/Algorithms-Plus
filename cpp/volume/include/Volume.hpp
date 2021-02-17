#ifndef __VOLUME_HPP
#define __VOLUME_HPP

#include <queue>
#include <deque>
#include <chrono>
#include <vector>
#include <utility>
#include <iostream>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "VolumeUtils.hpp"
#define RECORD                  // 录制输出
#define DIVX 1482049860         // OPENCV 格式编号

/**
 * 已知的一个问题（不想处理），运动时当竖直边恰好在光源竖直正方向时 / 或水平边在光源方向正水平方向时产生奇异性 
 * 处理（绕过），利用步长避免了此问题（避免了出现奇异性的位置）
 */

class Edge{
public:
    Edge(const cv::Point2f& _p1, const cv::Point2f& _p2, const cv::Point& light, bool ver):
        p1(_p1), p2(_p2), vertical(ver)
    {
        valid = true;
        centroid = (_p1 + _p2) / 2;
        orient = Edge::calcOrient<float>(light, centroid, ver);
        distance = calcDistance(centroid, light);
    }

    ~Edge(){;}
public:
    ///@brief 给定两束光线，更新边
    void updateSelf(const cv::Point& light, uchar** const occ, const cv::Point2f& v1, const cv::Point2f& v2);

    ///@brief 给定光源位置，计算两端点光线的方向向量（要归一化）
    void edgeVector(const cv::Point& pos, cv::Point2f& v1, cv::Point2f& v2) const;

    ///@brief 将自己绘制在图上
    void drawSelf(cv::Mat& src) const;

    ///@brief 根据光源位置计算边相对光源的方向 / 也可以计算block的相对位置
    template<typename Ty>
    static octOrient calcOrient(const cv::Point& light, const cv::Point_<Ty>& cen, bool vert = false);

    ///@brief 计算距离光源的距离
    static double calcDistance(const cv::Point2f& cen, const cv::Point& lit){
        cv::Point2f p = cen - cv::Point2f(lit.x, lit.y);
        return std::sqrt(p.ddot(p));
    }

    bool isValid() const {
        return this->valid;
    }

    void setValid(bool val = false) {
        this->valid = val;
    }

    double getDistance() const {
        return this->distance;
    }

    octOrient getOrient() const {
        return orient;
    }
    
    /// @brief 根据光线方向vec重新计算某个端点
    /// @param rebase_p1 是否重新计算p1
    void rebaseVertex(const cv::Point& light, uchar** const occ, const cv::Point2f& v1, const cv::Point2f& v2, bool rebase_p1);
public:
    cv::Point2f centroid;   //边界质心
    cv::Point2f p1;     // 端点1
    cv::Point2f p2;     // 端点2
private:
    bool valid;         // 边是否被遮挡
    bool vertical;     // 是否是竖直方向的边 
    double distance;    // 距离当前光源的距离
    octOrient orient;   // 相对光源的位置
};

struct EdgeCompFunctor{
    bool operator() (const Edge* const e1, const Edge* const e2) const {
        return e1->getDistance() > e2->getDistance();
    }
};

/// 2D体积光
class Volume{
public:
    Volume(int stone_num = 80);
    ~Volume();
public:
    ///@brief 单步移动
    void moveStep(DIRECT mv);

    ///@brief 平滑移动
    void moveSmooth(DIRECT mv, std::string win_name);

    void generateMap(int stone_num = 80);

    ///@brief 分块计算边界
    void getOutSideEdgeByBlock();

    ///@brief 按照距离pop，确定边界可见性以及重新计算边界端点
    void edgeTranverse();

    ///@brief 绘制2D体积光效果
    void render();

    ///@brief debug时绘制地图
    void debugDisplay(std::string win_name, bool render_flag = false);

    void reset(bool reset_pos = false){
        all_edges.clear();
        contours.clear();
        while (edges.empty() == false){
            edges.pop();
        }
        if (reset_pos){
            pos_occ == cv::Point(1, 1);
            pos_map == cv::Point(1.5 * BLOCK_SIZE, 1.5 * BLOCK_SIZE);
        }
    }

    /// 输入方向向量，计算与地图边界的交点 (可能5个也可能4个)
    void getRenderFrame(const Edge* const eg, const cv::Point2f& v1, const cv::Point2f& v2, std::vector<cv::Point>& contour) const;
private:
    static bool orientTagDueJudge(octOrient proj, octOrient cur){
        switch (proj){
            case DT: return (cur & BLOCK_UP) == 0;
            case DL: return (cur & BLOCK_LEFT) == 0;
            case DR: return (cur & BLOCK_RIGHT) != 0;
            case DB: return (cur & BLOCK_DOWN) != 0;
        }
    }

    void pushBackEdges(std::vector<Edge>& egs, int startx, int starty) const;

    // 根据边界 / 边界光线单位向量计算在地图边界上的解
    static void getBoundarySolution(const cv::Point2f& eg, const cv::Point2f& v, cv::Point& s);
public:
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeCompFunctor> edges;     // 边的小顶优先队列
private:
    cv::RNG* rng;
    cv::Mat map;            // 地图
    uchar** occ;            // 占位地图
    cv::Point pos_occ;      // 光源位置（占用地图）
    cv::Point pos_map;      // 光源位置（图像）
    std::vector<Edge> all_edges;                                            // 用于debug绘制
    std::vector<Edge> debug_vec;                                            // 用于debug绘制
    std::vector<std::vector<cv::Point> > contours;                          // 需要渲染的区域
    #ifdef RECORD
    std::string outPath = "/home/sentinel/volume.avi"; //+ std::string(argv[1]) + std::string(".avi");
    cv::Size sWH = cv::Size(1200, 900);
	cv::VideoWriter outputVideo;
    #endif
};

#endif  //__VOLUME_HPP