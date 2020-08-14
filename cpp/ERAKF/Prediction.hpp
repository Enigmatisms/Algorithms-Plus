/// 云台角度预测预实现-鼠标位置预测
/// @author hqy
/// @date 2020.5.5
#ifndef PREDICTION_HPP
#define PREDICTION_HPP
#include <iostream>
#include <deque>
#include <thread>
#include "src/Record.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#define MOUSE_TEST
//#define MOUSE_RECORD
#define RECORD_TEST

class Prediction{
public:
    Prediction();
    ~Prediction(){;}
public:
    void init(double cov);

    /**
     * @brief 单个角度预测
     * @param now_ang 当前云台位置
     * @param ang_w 预测的角速度
     * @param ang_b 预测的角加速度
     * @param ang_f 预测的急动度
     * @param dt 时间间隔
     */
    void predict(double now_ang, double &ang_w, double &ang_b, double &ang_f, double &dt);
public:
    double old_time;        // 上次时间
private:
    inline void calcMat(double dt, cv::Mat &A);

    ///=======================RKF优化========================///
    /**
     * @brief 基于Huber法的等价权矩阵求法
     * @param mes 观测(或非模型)矩阵
     * @param model 模型计算结果
     * @param cov 协方差矩阵
     * @param dst 输出
     */
    static void eqwMat(const cv::Mat &mes, const cv::Mat &model, const cv::Mat &cov, cv::Mat &dst);

    /// @brief 求对角阵的逆
    static inline void diagInv(const cv::Mat& src, cv::Mat& dst);

    /// @brief 自适应抗差解
    /// @param (w, b, f) 输出量: ang_w, ang_b, ang_f
    /// @param scale 比例放大因子
    void adaptiveSolve(double &w, double &b, double &f, double scale = 1.0);

    /// @brief 计算自适应因子
    /// @param rx 抗差解
    /// @param x CKF 预测
    /// @param precov CKF 先验协方差
    /// @return 自适应因子alpha
    static inline double adaptiveAlpha(const cv::Mat &rx, const cv::Mat &x, const cv::Mat &precov);
private:
    cv::KalmanFilter kf;
    cv::Mat measure;
    cv::Mat old_statex;     // x(k-1)的后验均值
    cv::Mat pnc;            // 运动模型协方差矩阵(由于抗差解使用的协方差矩阵是等加权)
    double old_ang;         // 上次角度
    double old_ang_w;       // 上次角速度
    double old_ang_b;       // 上次角加速度
    bool is_init;
    //===========抗差============
    cv::Mat temp1;
    cv::Mat temp2;
    cv::Mat temp3;
    cv::Mat temp4;
    cv::Mat r_state;        // 自适应抗差估计
    cv::Mat r_gain;         // 自适应抗差增益
};

class AngPre{
public:
    AngPre();
    ~AngPre(){
        ;
    }
public:
    bool predict(double now_yaw, double now_pitch, cv::Point2f &res, double dt = -1.0);    //返回一个点
    void queueInput(double yaw, double pitch);      // 存入队列

    /** 
     * @brief 限制预测，抑制超调
     * @param ratio 比例限制
     * @param d_ang 由预测角速度直接贡献的预测量
     * @param increment 由角加速度以及急动度贡献的预测量(高阶导致过于敏感？)
     * @return 抑制后的预测量
    */
    inline static double curbedPredict(double dt, double w, double b, double f, double ratio);
    
    #ifdef MOUSE_TEST
    /// @brief 绘制预测点以及实际位置点
    void drawPoints(cv::Mat &src, const cv::Point2f &pre, const cv::Point2f &act, bool success);

    /// 使用鼠标测试并绘制点(返回false时退出)
    bool testPredict(cv::Mat &src);
    #endif  //MOUSE_TEST

    //#if defined(MOUSE_RECORD) || defined(RECORD_TEST)
    //#endif  //MOUSE_RECORD
public:
    #if defined(MOUSE_RECORD) || defined(RECORD_TEST)
        Record<vec3> rcd;
    #endif  //MOUSE_RECORD || RECORD_TEST
private:
    Prediction yaw_pre;
    Prediction pitch_pre;
    std::deque<cv::Point2f> ang_que;
};

#endif //PREDICTION_HPP