#ifndef __VOLUME_UTILS_HPP
#define __VOLUME_UTILS_HPP
#include <map>
#include <opencv2/core.hpp>

#define VISUAL_X 40
#define VISUAL_Y 30
#define BLOCK_SIZE 30
#define NUMERICAL_ZERO 1e-4 // abs小于这个值的均认为是0
#define BLOCK_LEFT 0x03     // 边界orient与 0x03之后为0则可以加入
#define BLOCK_RIGHT 0x02    // 边界orient与 0x02后不为0则可以加入
#define BLOCK_UP 0x0c       // 边界orient与 0x0c后为0则可以加入
#define BLOCK_DOWN 0x08     // 边界orient与 0x08后不为0可以加入

/**
 * 2D平面体积光实现
 * 尝试采用如下方法进行实现：
 *  1000 * 800 随机视野，每个块30 * 30 pix大小
 *  对于初始的砖块，在是视野范围（砖块半径8范围内能见）
 *  在初始化地图后计算边界集合（需要融合）也就是一个unordered map结构
 *      首先，初始的外层边界都是确定的（地图生成完毕之后地图内的砖块相对位置就确定了），可以认为
 *      所有的边都是初始已经连接的，我们需要在所有的外侧边中，标记出所有面向光源的边，所有面向光源的边会形成新的边集合
 *      边集合中的边处理肯定是有先后顺序的，优先队列应该如何进行运作？什么样的边先出？
 *      思路必定是这样的：确定哪些边界是会出现的，最后所有会出现的边界进行填充即可
 *      从队列中当前距离最近的边开始计算，一条边界可以确定两条射线，也就是两个方向
 *      采用8叉搜索法（减少搜索数量）：
 *          假设某个边界在光源的正方向上（采用正方向的遍历搜索）
 *          如果不在正方向上，按照相对所在象限搜索
 *      使用边界夹角判定法，转化到极坐标下就很好判定了
 *          两个端点在阴影角度范围内，则这条边界设为 已经被遮挡
 *          其中一个端点在阴影角度范围内（不包括阴影角度），可能是部分遮挡的，需要重新计算其端点
 *          均不在阴影角度范围内，可以跳过
 *      由于存在距离优先性，边界是不会被重复计算的，所以已经尝试过投影的边界是需要从队列中pop的
 *      其后的边界也是：首先看是否完全被遮挡，完全被遮挡就直接pop，所以要写一个边的class或者struct
 *      关于渲染，阴影部分我个人想直接让其是黑色的，而可见部分到阴影是存在平凡反比衰减的
 *      可以直接渲染吗？在判定一个边界之后直接渲染边界遮住的所有部分
 *          个人认为直接渲染也不麻烦，需要计算在地图边界处的两个交点，与边界点合并，成为一个contour，直接保存这个contour
 *          最后将所有contour并行地使用fillContour填充即可
 */

enum DIRECT{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    IDLE = 4
};

enum octOrient{
    TL = 0x00,     // 左上
    DT = 0x01,     // 正上
    TR = 0x02,     // 右上
    DL = 0x04,     // 正左
    DR = 0x06,     // 正右
    BL = 0x08,     // 左下
    DB = 0x09,     // 正下
    BR = 0x0a      // 右下
};

extern const std::map<octOrient, cv::Scalar> colors;

extern const cv::Point dirs[4];

extern const int steps;


#endif  //__VOLUME_UTILS_HPP