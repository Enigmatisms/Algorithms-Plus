#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <chrono>
#include <deque>
#include "RobustError.hpp"

#define DEQUE_SIZE 20
#define ACC_BUF_MAX_SIZE 4      // 4帧最多50ms 所以可以认为加速度在这个时间内恒定

struct Msg {
    float x;
    float y;
    float z;
    Msg() {;}
    Msg(float _x, float _y, float _z):
        x(_x), y(_y), z(_z)
    {
        ;
    }
}; 

extern double coeff_vx;
extern double coeff_ax; 
extern double speed_thresh;     // 速度阈值
extern double acc_thresh;       // 加速度阈值

enum SimType {
    Tanh = 0,               // Tanh 近似方波
    Sinusoid = 1           // 正弦波
};

// 欧拉角转四元数 RPY 定义
static Eigen::Quaterniond angle2Quat(float r, float p, float y, bool is_rad = false){
    if (is_rad == false){
        r *= 0.0174533;       // pi / 180 = 0.0174533
        p *= 0.0174533;
        y *= 0.0174533;     
    }
    return Eigen::Quaterniond (
        cosf(r / 2) * cosf(p / 2) * cosf(y / 2) + sinf(r / 2) * sinf(p / 2) * sinf(y / 2),  // w
        sinf(r / 2) * cosf(p / 2) * cosf(y / 2) - cosf(r / 2) * sinf(p / 2) * sinf(y / 2),  // x
        cosf(r / 2) * sinf(p / 2) * cosf(y / 2) + sinf(r / 2) * cosf(p / 2) * sinf(y / 2),  // y
        cosf(r / 2) * cosf(p / 2) * sinf(y / 2) - sinf(r / 2) * sinf(p / 2) * cosf(y / 2)   // z
    );
}

class Predict {
public:
    Predict();
    ~Predict();
public:
    /// @brief 输入当前敌方位姿，根据pitch, yaw进行投影，在世界坐标系下进行预测，预测结果反投影回到相机坐标系，便于电控接收
    /// @return true 预测成功,使用预测值,否则使用装甲板
    bool translatePredict(const cv::Point3f& t_cam, const Msg& msg, Eigen::Vector3d& cam_p);

    /// @brief 模拟目标
    Eigen::Vector3d simulateTarget(double z, enum SimType type);

    /// @brief 预测绘制
    void drawProjected(cv::Mat& src, const Eigen::Vector3d& tar, const Eigen::Vector3d& pre);

    /// @brief 切换目标需要进行reset
    void reset();
private:
    /// 根据当前值计算观测值，需要计时
    void calcObvserved(const Eigen::Vector3d& pw, Vector6d& obs, double dt, double lambda = 0.5);

    /// 计算状态转移矩阵
    void calcStateTransit(double dt);

    void noiseDEstimate(const Vector6d& obs);
    
    /// 对结果进行后续处理
    /// 1. 减速预测削减
    /// 2. 稳态抖动减小
    void postProcess(Eigen::Vector3d& predict);

    static void project2World(
        const cv::Point3f& t_cam,
        float pit, float yaw,
        Eigen::Vector3d& pw,
        Eigen::Quaterniond& c2w
    );
private:
    bool init;
    Matrix6d A;             // 状态转移
    Matrix6d P;             // 状态转移协方差
    Matrix6d Q;             // 状态转移误差
    Matrix6d R;             // 观测误差
    Vector6d state_post;    // 后验状态
    Vector6d state_pre;     // 先验状态
    Vector6d old_obs;
    std::chrono::_V2::system_clock::time_point saved_time_point;
    std::chrono::_V2::system_clock::time_point init_point;
    Eigen::Matrix3d this_K;
    cv::RNG* rng;
    std::fstream file;

    size_t inov_cnt;
    std::deque<double> innovation[6];           // 新息
    std::deque<double> acc_buff[2];             // 两轴加速度缓冲区
    ceres::Solver::Options state_opts;

    double direct;
    double air_k;
};