#include "../include/Predict.hpp"

const Matrix6d I6d = Matrix6d::Identity();

inline static double chronoGetTime(std::chrono::system_clock::time_point& old) {
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> interval = now - old;
    old = now;
    return interval.count();            // 返回以秒为单位的时间间隔
}

#define _GRAVITY 9.79
static float BulletModel(float x, float v, float angle, float k)
{
    return x * _GRAVITY / (k * v * cosf(angle)) + tanf(angle) * x +
           1 / (k * k) * _GRAVITY * logf(1 - k * x / (v * cosf(angle)));
}

static float calcTime(float x, float v, float angle, float k)
{
    return (-1 / (k)*logf(1 - k * x / (v * cosf(angle))));
}


#define _RAD2DEG 57.29578
static void solve(const Vector6d &state, const Msg &msg, Eigen::Vector3d &pos, float k, double dt_s)
{
    Eigen::Vector2d now = state.block<2, 1>(0, 0);
    float y_temp, y_act, dy, delta_t = 0.0, old_delta = 0.0;
    float angle = msg.y / _RAD2DEG, start_angle = angle;
    float dist = now.norm() / 1000, y_pos = -pos(1) / 1000; // 加负号的原因是，相机坐标系向下为正，而实际弹道解算应该向上为正
    y_temp = dist * tanf(angle);                            // y_temp 为枪管指向的y位置, y_pos 为目标所在的y位置
    double now0 = now(0), now1 = now(1);
    for (int i = 0; i < 25; i++)
    {
        angle = atan2f(y_temp, dist);

        y_act = BulletModel(dist, msg.z, angle, k);
        dy = y_pos - y_act;
        y_temp += dy;

        delta_t = calcTime(dist, msg.z, angle, k);
        now(0) = now0 + delta_t * state(2) * coeff_vx + 0.5 * delta_t * delta_t * state(4) * coeff_ax;
        now(1) = now1 + delta_t * state(3) * 3 + 0.5 * delta_t * delta_t * state(5) * 1;
        dist = now.norm() / 1000;

        if (fabsf(delta_t - old_delta) < 0.01 && fabsf(dy) < 0.001)
        {
            break;
        }
        old_delta = delta_t;
    }
    pos(0) = 600 * tanh((now(0) - state(0)) / 500);             // 输出是增量，增量需要进行后续的修正
    pos(2) = 600 * tanh((now(1) - state(1)) / 500);
    pos(1) = 0.0;
}

// =========================== 非static主要预测逻辑 ===============================

Predict::Predict() {
    this_K << 
        1776.67168581218, 0, 720,
        0, 1778.59375346543, 540,
        0, 0, 1;
    median.resize(4);
    reset();
    inov_cnt = 0;
    direct = 1.0;
    rng = new cv::RNG(std::chrono::system_clock::now().time_since_epoch().count());
    init_point = std::chrono::system_clock::now();
    file.open("../data/data_robust.txt", std::ios::out);
    air_k = 0.00831;

    state_opts.linear_solver_type = ceres::DENSE_QR;
    state_opts.minimizer_type = ceres::LINE_SEARCH;
    state_opts.line_search_direction_type = ceres::LBFGS;
    state_opts.minimizer_progress_to_stdout = false;
    state_opts.max_linear_solver_iterations = 50;
    state_opts.max_num_iterations = 50;
    state_opts.function_tolerance = 1e-6;
    state_opts.logging_type = ceres::SILENT;
}

Predict::~Predict() {
    file.close();
    delete rng;
}

void Predict::reset() {
    init = false;
    A = Matrix6d::Identity();
    P.setZero();
    Vector6d tmp;
    R = Matrix6d::Identity();
    Q = Matrix6d::Identity() * 64;
    old_obs = Vector6d::Zero();
    state_post = Vector6d::Zero();
    state_pre = Vector6d::Zero();
    saved_time_point = std::chrono::system_clock::now();
    acc_buff[0].clear();
    acc_buff[1].clear();
    for (size_t i = 0; i < median.size(); i++) {
        median[i].clear();
    } 
}

// 4个state x, y, vx, vy
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

void Predict::calcObvserved(const Eigen::Vector3d& pw, Vector6d& obs, double dt, double lambda){
    obs(0) = pw(0);     // x->x
    obs(1) = pw(2);     // x->y
    obs(2) = (obs(0) - old_obs(0)) * lambda + old_obs(2) * (1 - lambda);
    obs(3) = (obs(1) - old_obs(1)) * lambda + old_obs(3) * (1 - lambda);
    obs(4) = (obs(2) - old_obs(2)) * (lambda - 0.1) + old_obs(4) * (1.1 - lambda);      // 加速度需要更加平滑
    obs(5) = (obs(3) - old_obs(3)) * (lambda - 0.1) + old_obs(5) * (1.1 - lambda);
    old_obs = obs.eval();
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
    calcObvserved(pw, obs, dt_s, 0.3);
    calcStateTransit(dt_s);
    P = A * P * A.transpose() + Q;
    Matrix6d invPr = (P + R).ldlt().solve(I6d);
    Matrix6d K = P * invPr;
    // Huber函数的抗差KF（没有解析解，所以优化问题需要ceres）
    Mat12d Exp = Mat12d::Zero();
    Exp.block<6, 6>(0, 0) = P;
    Exp.block<6, 6>(6, 6) = R;
    Mat12d S = Exp.llt().matrixL();
    Mat12d Sinv = S.ldlt().solve(Mat12d::Identity()); //S^{-1}
    HalfMat12d X;
    X.block<6, 6>(0, 0) = Sinv.block<6, 6>(0, 0) + Sinv.block<6, 6>(0, 6); // 分块矩阵乘法
    X.block<6, 6>(6, 0) = Sinv.block<6, 6>(6, 0) + Sinv.block<6, 6>(6, 6);
    Vec12d tmp;
    tmp.block<6, 1>(0, 0) = state_post;
    tmp.block<6, 1>(6, 0) = obs;
    Vec12d Y = Sinv * tmp;
    state_pre = A * state_post;
    Vector6d old_state = state_post;
    Vector6d inov = obs - state_pre;
    // 并行进行状态与协方差估计，但是个人感觉这应该也是挺快的
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            ceres::Problem state_prob;
            ceres::CostFunction *cost_func = RobustStateProb::Create(X, Y, 1);
            state_prob.AddResidualBlock(cost_func, nullptr, state_post.data());
            ceres::Solver::Summary summary;
            ceres::Solve(state_opts, &state_prob, &summary);
        }
        #pragma omp section
        {
            noiseDEstimate(inov);           // 噪声估计是需要等待队列装满才能开始的
        }
    }
    state_post(2) = state_post(2) * 0.25 + old_state(2) * 0.75;
    state_post(3) = state_post(3) * 0.25 + old_state(3) * 0.75;
    state_post(4) = state_post(4) * 0.25 + old_state(4) * 0.75;
    state_post(5) = state_post(5) * 0.25 + old_state(5) * 0.75;
    for (int i = 0; i < 4; i++) {
        median[i].emplace_back(state_post(i + 2));
        if (median[i].size() > 3) {//median blur
            median[i].pop_front();
            std::vector<double> tmp;
            tmp.assign(median[i].begin(), median[i].end());
            std::nth_element(tmp.begin(), tmp.begin() + 1, tmp.end());
            state_post(i + 2) = tmp[1];
        }
    }
    P -= K * (P + R) * K.transpose();
    // state_post 是当前对状态的估计，那么只需要当前加速度 / 速度 / 位置进行双迭代 (x, y, vx, vy, ax, ay)
    Eigen::Vector3d result(state_post(0), pw(1), state_post(1)), delta_pos(0, pw(1), 0);
    solve(state_post, msg, delta_pos, air_k, dt_s); // 恒定速度的双迭代
    // 此处需要加上预测之后的后续处理，比如说：1.操作手系数控制 2. 减速预测削减 3. 稳态抖动消除
    postProcess(delta_pos);
    result += delta_pos;
    cam_p = c2w.conjugate() * result;            // 从result（预测之后的世界坐标）转化为相机坐标
    file << t_cam.x << ',' << t_cam.z << "," << cam_p.x() << "," << cam_p.z();      // 0 1 2 3
    file << ","<< state_post(2) << "," << state_post(3) << "," << state_post(4) * coeff_ax / coeff_vx << "," << state_post(5) << std::endl;     // 4 5 6 7
    return true;
}

void Predict::postProcess(Eigen::Vector3d& predict) {
    if (init == false) {                // 没有完全初始化，此时直接削减预测值为1/4
        predict *= 0.25;                // 魔法参数
        return;
    }
    double mean_a[2] = {0., 0.};
    for (int i = 0; i < 2; i++) {
        acc_buff[i].emplace_back(state_post(4 + i));
        if (acc_buff[i].size() > ACC_BUF_MAX_SIZE) {
            acc_buff[i].pop_front();
        }
        mean_a[i] = std::accumulate(acc_buff[i].cbegin(), acc_buff[i].cend(), mean_a[i]);
        mean_a[i] /= double(ACC_BUF_MAX_SIZE);
    }
    double vx_diff = speed_thresh - std::abs(state_post(2));
    double vz_diff = speed_thresh - std::abs(state_post(3));
    double ax_diff = acc_thresh - std::abs(mean_a[0]);
    double az_diff = acc_thresh - std::abs(mean_a[1]);
    if ( vx_diff < 0 || vz_diff < 0 || ax_diff < 0 || az_diff < 0) {        // 有较大的速度或者加速度
        // 由于电控对于加速跟不上，而减速是能跟上的，为了避免超调，减速时需要减小预测量（以减小超调）
        if ((state_post(2) > speed_thresh && mean_a[0] < acc_thresh) ||
            (state_post(2) < -speed_thresh && mean_a[0] > -acc_thresh)
        ) {
            double diff = std::abs(acc_thresh - mean_a[0]) + 1;
            predict(0) = predict(0) * 1 / (std::log(diff) + 1);
        }
        if ((state_post(3) > speed_thresh && mean_a[1] < acc_thresh) ||
            (state_post(3) < -speed_thresh && mean_a[1] > -acc_thresh)
        ) {
            double diff = std::abs(acc_thresh - mean_a[1]) + 1;
            predict(2) = predict(2) * 1 / (std::log(diff) + 1);
        }
    }
    else {      // 速度与加速度都很小，则削减预测量
        double suppress = exp(- vx_diff/speed_thresh) * exp(- vz_diff/speed_thresh)
            * exp(- ax_diff / acc_thresh) * exp(- az_diff / acc_thresh);
        predict *= suppress;
    }
}

// 一个CA模型
void Predict::calcStateTransit(double dt) {
    double vx = state_post(2), vy = state_post(3), ax = state_post(4), ay = state_post(5);
    A << 
    1, 0, dt * vx, 0, ax * dt * dt * 0.5, 0,
    0, 1, 0, dt * vy, 0, ay * dt * dt * 0.5,
    0, 0, 1, 0, ax * dt, 0,
    0, 0, 0, 1, 0, ay * dt,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;
}

Eigen::Vector3d Predict::simulateTarget(double z, enum SimType type) {
    double now = std::chrono::system_clock::now().time_since_epoch().count() / 1e6;
    now -= init_point.time_since_epoch().count() / 1e6;
    double freq = 0.003;
    double x = 0.0;
    if (type == Tanh) {
        double res = direct * std::tanh(0.007 * now - 3);
        if (res >= 0.99995) {
            init_point = std::chrono::system_clock::now();
            direct = -1;
        }
        else if (res <= -0.99995){
            init_point = std::chrono::system_clock::now();
            direct = 1;
        }
        x = res * 1000;
    }
    else {
        x = 1000 * std::sin(freq * now);
    }
    
    printf("X now is: %lf, now t is %lf, freq: %f\n", x, now, freq);
    return Eigen::Vector3d(x, 0, z) + z / 200 * Eigen::Vector3d::Random();
}

void Predict::drawProjected(cv::Mat& src, const Eigen::Vector3d& tar, const Eigen::Vector3d& pre) {
    Eigen::Vector3d ctar = tar / tar(2);
    Eigen::Vector3d tpj = this_K * ctar;
    cv::circle(src, cv::Point(tpj(0), tpj(1)), 10, cv::Scalar(0, 0, 255), -1);
    Eigen::Vector3d ptar = pre / pre(2);
    Eigen::Vector3d ppj = this_K * ptar;
    cv::circle(src, cv::Point(ppj(0), ppj(1)), 10, cv::Scalar(0, 255, 0), -1);
}

template<typename T, template <typename ELEM, typename Alloc = std::allocator<ELEM> > typename Contain>
T findMedian(const Contain<T>& dq) {
    std::vector<T> tmp;
    size_t half = dq.size() / 2;
    tmp.assign(dq.begin(), dq.end());
    std::nth_element(tmp.begin(), tmp.begin() + half, tmp.end());
    return tmp[half];
}

const double huber_bounds[6] = {1.0, 0.2, 1.0, 0.2, 0.5, 0.1};
void Predict::noiseDEstimate(const Vector6d& inov) {
    for (int i = 0; i < 6; i++)
        innovation[i].emplace_back(inov(i));
    if (inov_cnt < DEQUE_SIZE) {     // innovation cnt smaller than 7
        inov_cnt++;
        return;
    }
    else {
        for (int i = 0; i < 6; i++)
            innovation[i].pop_front();
        double res[6];
        memset(res, 0.1, 6 * sizeof(double));
        #pragma omp parallel for num_threads(2)
        for (int i = 0; i < 6; i++) {
            double med = findMedian(innovation[i]);
            std::vector<double> diff;
            for (double inov: innovation[i])
                diff.emplace_back(std::abs(inov - med) / 0.6745);
            double d = findMedian(diff) + 1e-5;
            RobustCovProb::MakeProb2Solve(innovation[i], state_opts, d, huber_bounds[i], &res[i]);
        }
        for (int i = 0; i < 6; i++) {
            R(i, i) = 0.0;
            for (double val: innovation[i]) {
                double inter = val - res[i];
                R(i, i) += std::pow(inter, 2);
            }
            R(i, i) = std::sqrt(R(i, i)) / DEQUE_SIZE;
        }
        Q = 64 * R;
    }
}