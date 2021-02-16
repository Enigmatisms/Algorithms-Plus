#ifndef __VOLUME_HPP
#define __VOLUME_HPP

#include <queue>
#include <deque>
#include <chrono>
#include <vector>
#include <utility>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "VolumeUtils.hpp"
/**
 * 有两种解决办法 1. 本方法下计算每一条边属于哪个地图块上4条边中的哪一条（繁琐）
 * 2. 当然也可： 按照块计算边，开始时就已经计算好可能被照射的边了
 * 算法设计需要大改，首先优先队列应该存指针（因为优先队列没有迭代能力，不可随机访问）
 *      关于按照块来计算，如何是实现并行化？简单的想法就是地图分成4块，分块并行化操作 （omp section）
 *      每个块直接先计算其相对光源的位置，根据这个算出来的位置可以计算它外层边哪是应该被包括的
 *      计算出边后可视化以下看看结果
 *      根据结果，队列进行计算（队列存指针），指针指向的本体不销毁，存在vector内
 *      需要添加inspected flag，最后的八叉搜索
 *      8叉搜索怎么实现呢？可以这样：每个边再记录一下相对位置（8个方向的标识），此后遍历整个边vector时可以并行更新
 *      由于只需要相对光源的8个90度方向即可。
*/
class Edge{
public:
    Edge(const cv::Point2f& _p1, const cv::Point2f& _p2, const cv::Point& light, bool ver):
        p1(_p1), p2(_p2), veritical(ver)
    {
        valid = true;
        popped = false;
        centroid = (_p1 + _p2) / 2;
        orient = Edge::calcOrient<float>(light, centroid);
        distance = calcDistance(p1, p2, cv::Point(1, 1));
    }

    ~Edge(){;}
public:
    ///@brief 给定光源位置，计算两端点光线角度
    void edgeAngles(const cv::Point& pos, float& a1, float& a2) const;

    ///@brief 将自己绘制在图上
    void drawSelf(cv::Mat& src) const;

    ///@brief 根据光源位置计算边相对光源的方向 / 也可以计算block的相对位置
    template<typename Ty>
    static octOrient calcOrient(const cv::Point& light, const cv::Point_<Ty>& cen);

    ///@brief 计算距离光源的距离
    static double calcDistance(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point& lit){
        cv::Point2f p = (p1 + p2) / 2;
        p.x -= lit.x;
        p.y -= lit.y;
        return std::sqrt(p.ddot(p));
    }

    bool isValid() const {
        return this->valid;
    }

    double getDistance() const {
        return this->distance;
    }

    octOrient& getOrient() {
        return orient;
    }

    const cv::Point2f& getCentroid(){
        return this->centroid;
    }
    
    /// @brief 根据光线方向vec重新计算某个端点
    /// @param rebase_p1 是否重新计算p1
    void rebaseVertex(const cv::Point2f& vec, bool rebase_p1 = false);

private:
    bool valid;         // 边是否被遮挡
    bool veritical;     // 是否是竖直方向的边 
    bool popped;        // 是否出队
    double distance;    // 距离当前光源的距离
    octOrient orient;   // 相对光源的位置
    cv::Point2f p1;     // 端点1
    cv::Point2f p2;     // 端点2
    cv::Point2f centroid;   //边界质心
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
    void move(DIRECT mv);

    void generateMap(int stone_num = 80);

    ///@brief 分块计算边界
    void getOutSideEdgeByBlock();

    ///@brief 按照距离遍历，确定边界可见性以及重新计算边界端点
    void edgeTranverse(){;}

    ///@brief 绘制2D体积光效果
    void render(){;}

    ///@brief debug时绘制地图
    void debugDisplay();

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
private:
    void pushBackEdges(std::vector<Edge>& egs, int startx, int starty) const;
public:
    cv::RNG* rng;
    uchar** occ;            // 占位地图
    cv::Mat map;            // 地图
    cv::Point pos_occ;      // 光源位置（占用地图）
    cv::Point pos_map;      // 光源位置（图像）
    std::vector<Edge> all_edges;                                            // 用于debug绘制
    std::vector<Edge> debug_vec;                                            // 用于debug绘制
    std::vector<std::vector<cv::Point> > contours;                          // 需要渲染的区域
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeCompFunctor> edges;     // 边的小顶优先队列
};

#endif  //__VOLUME_HPP