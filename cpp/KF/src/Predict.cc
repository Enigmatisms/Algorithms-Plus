#include "../include/Predict.hpp"

const Matrix6d I6d = Matrix6d::Identity();

inline static double chronoGetTime(std::chrono::system_clock::time_point& old) {
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> interval = now - old;
    old = now;
    return interval.count();            // 返回以秒为单位的时间间隔
}

#define K0 0.00831
#define _GRAVITY 9.79
static float BulletModel(float x, float v, float angle)
{
    return x * _GRAVITY / (K0 * v * cosf(angle)) + tanf(angle) * x +
           1 / (K0 * K0) * _GRAVITY * logf(1 - K0 * x / (v * cosf(angle)));
}

static float calcTime(float x, float v, float angle)
{
    return (-1 / (K0) * logf(1 - K0 * x / (v * cosf(angle))));
}

#define _RAD2DEG 57.29578
static void solve(const Vector6d& state, const Msg& msg, Eigen::Vector3d& pos)
{
    Eigen::Vector2d now = state.block<2, 1>(0, 0);
    float y_temp, y_act, dy, delta_t = 0.0, old_delta = 0.0;
    float angle = msg.y / _RAD2DEG, start_angle = angle;
    float dist = now.norm() / 1000, y_pos = -pos(1) / 1000;      // 加负号的原因是，相机坐标系向下为正，而实际弹道解算应该向上为正
    float t_sum = 0;
    y_temp = dist * tanf(angle);                    // y_temp 为枪管指向的y位置, y_pos 为目标所在的y位置
    for (int i = 0; i < 25; i++)
    {
        angle = atan2f(y_temp, dist);
        y_act = BulletModel(dist, msg.z, angle);
        delta_t = calcTime(dist, msg.z, angle);
        t_sum += delta_t;
        dy = y_pos - y_act;
        y_temp += dy;
        if (fabsf(delta_t - old_delta) < 0.01 && fabsf(dy) < 0.001)
        {
            break;
        }
        old_delta = delta_t;
    }
    pos(0) = now(0) + 1.0 * t_sum * state(2) + 0.1 * t_sum * t_sum * state(4);
    pos(2) = now(1) + 1.0 * t_sum * state(3) + 0.1 * t_sum * t_sum * state(5);
}

// =========================== 非static主要预测逻辑 ===============================

Predict::Predict() {
    this_K << 
        1776.67168581218, 0, 720,
        0, 1778.59375346543, 540,
        0, 0, 1;
    reset();
    rng = new cv::RNG(std::chrono::system_clock::now().time_since_epoch().count());
}

Predict::~Predict() {
    delete rng;
}

void Predict::reset() {
    init = false;
    A = Matrix6d::Identity();
    Matrix6d rd = Matrix6d::Random();
    P = rd * rd.transpose();
    Q = 400 * rd * rd.transpose();
    R = 0.1 * rd * rd.transpose();
    state_post = Vector6d::Zero();
    state_pre = Vector6d::Zero();
    saved_time_point = std::chrono::system_clock::now();
}

// 6个state x, y, vx, vy, ax, ay;
// 相机系 根据云台位姿 投影 到 世界坐标系下 由于 在真实的比赛环境下，可能出现高度不统一的情况，所以得到的pw为3D位姿 存在z轴
// 电控的初始位置（pitch正前方，yaw为磁力计零位）对应的相机坐标系为世界坐标系（不考虑原点平移）
void Predict::project2World(
    const cv::Point3f& t_cam,
    float pit, float yaw,
    Eigen::Vector3d& pw,
    Eigen::Quaterniond& c2w
){
    c2w = angle2Quat(pit, - yaw, 0.0);
    Eigen::Vector3d cam_t(t_cam.x, t_cam.y, t_cam.z);
    pw = c2w * cam_t;
}

void Predict::calcObvserved(const Eigen::Vector3d& pw, Vector6d& obs, double dt, double lambda) const {
    obs(0) = pw(0);     // x->x
    obs(1) = pw(2);     // x->y
    obs(2) = (obs(0) - state_post(0)) * lambda + state_post(2) * (1 - lambda);
    obs(3) = (obs(1) - state_post(1)) * lambda + state_post(3) * (1 - lambda);
    obs(4) = (obs(2) - state_post(2)) * lambda + state_post(4) * (1 - lambda);
    obs(5) = (obs(3) - state_post(3)) * lambda + state_post(5) * (1 - lambda);
}

bool Predict::translatePredict(const cv::Point3f& t_cam, const Msg& msg, Eigen::Vector3d& cam_p) {
    Eigen::Vector3d pw;
    Vector6d obs;
    Eigen::Quaterniond c2w;
    project2World(t_cam, msg.y, msg.x, pw, c2w);        // pw相当于当前观测
    if (init == false) {
        saved_time_point = std::chrono::system_clock::now();
        state_post(0) = pw(0);
        state_post(1) = pw(2);
        init = true;
        return false;
    }
    double dt_s = chronoGetTime(saved_time_point);      // 根据上一次保存的时间计算以秒为单位的时间间隔
    calcObvserved(pw, obs, dt_s, 0.5);
    calcStateTransit(dt_s);
    P = A * P * A.transpose() + Q;
    Matrix6d invPr = (P + R).ldlt().solve(I6d);
    Matrix6d K = P * invPr;
    if (false) {                                     // 传统KF
        state_pre = A * state_post;                 // 没有中间控制量，注意state_post是上次预测估计的输出
        // pw 也就是 pw(0) = x(车右方), pw(1) = y(竖直向下), pw(2) = z 车直线向前
        // 根据上次预测的结果，根据时间，推算当前应该在什么位置
        state_post = state_pre + K * (obs - state_pre);
        P = (I6d - K) * P;
    }
    else {              // Huber函数的抗差KF（没有解析解，所以优化问题需要ceres）
        Mat12d Exp = Mat12d::Zero();
        Exp.block<6, 6>(0, 0) = P;
        Exp.block<6, 6>(6, 6) = R;
        Mat12d S = Exp.llt().matrixL();
        Mat12d Sinv = S.ldlt().solve(Mat12d::Identity()); //S^{-1}
        HalfMat12d X;
        X.block<6, 6>(0, 0) = Sinv.block<6, 6>(0, 0) + Sinv.block<6, 6>(0, 6);  // 分块矩阵乘法
        X.block<6, 6>(6, 0) = Sinv.block<6, 6>(6, 0) + Sinv.block<6, 6>(6, 6);
        Vec12d tmp;
        tmp.block<6, 1>(0, 0) = state_post;
        tmp.block<6, 1>(6, 0) = obs;
        Vec12d Y = Sinv * tmp;
        ceres::Problem state_prob;
        ceres::CostFunction* cost_func = RobustStateProb::Create(X, Y, 1);
        state_prob.AddResidualBlock(cost_func, nullptr, state_post.data());
        ceres::Solver::Options opts;
        opts.linear_solver_type = ceres::DENSE_QR;
        opts.minimizer_type = ceres::LINE_SEARCH;
        opts.line_search_direction_type = ceres::LBFGS;
        opts.minimizer_progress_to_stdout = true;
        opts.max_linear_solver_iterations = 50;
        opts.function_tolerance = 1e-6;
        ceres::Solver::Summary summary;
        ceres::Solve(opts, &state_prob, &summary);
        // P = (X.transpose() * X).ldlt().solve(Matrix6d::Identity());
        P -= K * P;
    }
    // state_post 是当前对状态的估计，那么只需要当前加速度 / 速度 / 位置进行双迭代 (x, y, vx, vy, ax, ay)
    // 双迭代在这里进入
    Eigen::Vector3d result(0, pw(1), 0);                        // 先不考虑非平面运动 (pw(1)是竖直方向的)
    solve(state_post, msg, result);                             // 恒定加速度 / 速度的双迭代
    cam_p = c2w.conjugate() * result;                           // 从result（预测之后的世界坐标）转化为相机坐标
    return true;
}

// 一个CA模型
void Predict::calcStateTransit(double dt) {
    double vx = state_post(2), vy = state_post(3), ax = state_post(4), ay = state_post(5);
    A << 
    1, 0, dt * vx, 0, 0.5 * ax * dt * dt, 0,
    0, 1, 0, dt * vy, 0, 0.5 * ax * dt * dt,
    0, 0, 1, 0, ax * dt, 0,
    0, 0, 0, 1, 0, ay * dt,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;
}

Eigen::Vector3d Predict::simulateTarget(double z) {
    double now = std::chrono::system_clock::now().time_since_epoch().count() / 1e6;
    double freq = 0.001;
    double x = 2000 * std::sin(freq * now);
    printf("X now is: %lf, now t is %lf, freq: %f\n", x, now, freq);
    return Eigen::Vector3d(x, 0, z) + z / 300 * Eigen::Vector3d::Random();
}

void Predict::drawProjected(cv::Mat& src, const Eigen::Vector3d& tar, const Eigen::Vector3d& pre) {
    Eigen::Vector3d ctar = tar / tar(2);
    Eigen::Vector3d tpj = this_K * ctar;
    cv::circle(src, cv::Point(tpj(0), tpj(1)), 10, cv::Scalar(0, 0, 255), -1);
    Eigen::Vector3d ptar = pre / pre(2);
    Eigen::Vector3d ppj = this_K * ptar;
    cv::circle(src, cv::Point(ppj(0), ppj(1)), 10, cv::Scalar(0, 255, 0), -1);
}