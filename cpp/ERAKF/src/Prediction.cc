#include "../Prediction.hpp"

Prediction::Prediction(){
    ;
    //CA 扩展模型(4*4转移矩阵，假设pitch yaw是独立的)（考虑速度，加速度，急动度）;
}

void Prediction::init(double cov){
    kf.init(4, 4, 0, CV_64F);           // 转移矩阵4, 观测矩阵4, 无控制量
    cv::RNG rng;
    kf.transitionMatrix = (cv::Mat_<double>(4, 4) <<
            1, 11, 60.5, 443.67,
            0,  1, 11.0,   60.5,
            0,  0,  1.0,   11.0,
            0,  0,    0,      1);
    cv::setIdentity(kf.errorCovPost);
    cv::setIdentity(kf.measurementMatrix);
    pnc = cv::Mat::zeros(4, 4, CV_64F);
    cv::Scalar tr;                      //协方差阵迹元素
    for(int i = 0; i < 4; ++i){
        tr[i] = cov + 20 * i;
    }
    cv::setIdentity(pnc, 2 * tr);
    cv::setIdentity(kf.processNoiseCov, 2 * tr);
    cv::setIdentity(kf.measurementNoiseCov, tr);
    rng.fill(kf.statePre, cv::RNG::UNIFORM, 0, 1);
    rng.fill(kf.statePost, cv::RNG::UNIFORM, 0, 1);
    measure = cv::Mat::zeros(4, 1, CV_64F);
    old_statex = cv::Mat::zeros(4, 1, CV_64F);
    kf.predict();
    is_init = false;

    temp1 = cv::Mat::zeros(4, 4, CV_64F);
    temp2 = cv::Mat::zeros(4, 4, CV_64F);
    temp3 = cv::Mat::zeros(4, 4, CV_64F);
    temp4 = cv::Mat::zeros(4, 4, CV_64F);
}


void Prediction::calcMat(double dt, cv::Mat &A){
    A = (cv::Mat_<double>(4, 4) <<
            1, dt, dt*dt/2, pow(dt, 3)/3,
            0,  1,      dt,    dt*dt/2,
            0,  0,       1,         dt,
            0,  0,       0,         1);
}

void Prediction::predict(double now_ang, double &ang_w, double &ang_b, double &ang_f, double &dt){
    ///TODO: debug，实际情况无需判断dt(delta_t当dt为正值时直接使用dt)
    double now_time = cv::getTickCount(),
        delta_t = dt > 0 ? dt : (now_time - old_time) / cv::getTickFrequency() * 1000.0,
        omega = (now_ang - old_ang) / delta_t,
        beta = (omega - old_ang_w) / delta_t;
    printf("Delta t is %f\n", delta_t);
    if(is_init == false){
        old_time = now_time;        //若dt > 0,则此项不会被用到
        old_ang = now_ang;
        old_ang_w = omega;
        old_ang_b = 0.0;
        dt = 0.0;
        is_init = true;
        return;
    }
    if(delta_t > 1.0){          //delta_t 计算无误才能预测
        calcMat(delta_t, kf.transitionMatrix);
        kf.statePost.copyTo(old_statex);            // 保存一份上一历元的状态
        kf.predict();
        measure.at<double>(0) = now_ang;
        measure.at<double>(1) = omega;
        measure.at<double>(2) = beta;
        measure.at<double>(3) = (beta - old_ang_b) / delta_t;
        old_time = now_time;
        old_ang = now_ang;
        old_ang_w = omega;
        old_ang_b = beta;

        // 替换噪声协方差阵为等价权阵

        eqwMat(measure, kf.measurementMatrix * kf.statePre, pnc, kf.processNoiseCov);  
        kf.correct(measure);
        //ang_w = kf.statePost.at<double>(1);
        //ang_b = kf.statePost.at<double>(2);
        //ang_f = kf.statePost.at<double>(3);
        adaptiveSolve(ang_w, ang_b, ang_f, 1.5);     // 自适应解
        dt = delta_t / 2 + 2;                 // 消除滞后影响并且超前1ms预测(可能造成超调)
    }
}

///=========================RKF===========================///

void Prediction::adaptiveSolve(double &w, double &b, double &f, double scale){
    /// kf.statePost 是抗差估计, statePre则是未经处理的CKF先验估计

    double alpha = adaptiveAlpha(kf.statePost, kf.statePre, kf.errorCovPre);
    temp1 = kf.processNoiseCov * kf.measurementMatrix;      // 等价权(第一项)是对角阵
    cv::Mat x_eqw = cv::Mat::zeros(4, 4, CV_64F);
    diagInv(kf.errorCovPre, x_eqw);                // 先验误差协方差的逆为其等价权
    temp2 = cv::Mat::zeros(4, 4, CV_64F);

    /// first part: (Ck'*Wk*Ck + a * inv(Pk_))
    cv::gemm(temp1, kf.measurementMatrix, 1, x_eqw, alpha, temp2, cv::GEMM_1_T);

    temp3 = x_eqw * kf.statePre;
    temp4 = cv::Mat::zeros(4, 4, CV_64F);
    cv::gemm(temp1, measure, 1, temp3, alpha, temp4, cv::GEMM_1_T);

    /// xk_hat = inv(Ck'*Wk*Ck + a * inv(Pk_)) * (Ck'*Wk*zk + a * inv(Pk_) * xk)
    cv::solve(temp2, temp4, r_state, cv::DECOMP_SVD);       //svd分解

    /// Pk_hat = inv(Ck'*Wk*Ck + a * inv(Pk_)) * scale ^ 2
    cv::solve(temp2, pow(scale, 2) * cv::Mat::eye(4, 4, CV_64F), kf.errorCovPost, cv::DECOMP_SVD);

    w = r_state.at<double>(1);
    b = r_state.at<double>(2);
    f = r_state.at<double>(3);
}

double Prediction::adaptiveAlpha(const cv::Mat &rx, const cv::Mat &x, const cv::Mat &precov){
    /// Huber法
    cv::Mat tem = rx - x;           // 抗差解向量减CKF预测结果
    printf("rkf: %lf, %lf %lf, %lf\n", tem.at<double>(0), tem.at<double>(1), tem.at<double>(2), tem.at<double>(3));
    double norm1 = tem.dot(tem);
    double dxk = sqrt(norm1 / cv::trace<double, 4, 4>(precov));     // CKF 先验协方差迹不会是0
    printf("Therefore norm1 and d_xk and trace = %lf, %lf, %lf\n", norm1, dxk, sqrt(cv::trace<double, 4, 4>(precov)));
    double c0 = 0.001, c1 = 0.002, res = 0;
    if(dxk <= c0){
        res = 1.0;
    }
    else if(dxk <= c1){
        res = c0 / dxk * pow((c1 - dxk) / (c1 - c0), 2);
    }
    else res = 0;
    printf("Adaptive alpha is %f\n", res);
    return res;
}

void Prediction::eqwMat(const cv::Mat &mes, const cv::Mat &model, const cv::Mat &cov, cv::Mat &dst){
    cv::Mat residual = (mes - model);
    double c = 1.5;
    for(int i = 0; i < 4; ++i){
        residual.at<double>(i, 1)= std::abs(residual.at<double>(i) / cov.at<double>(i, i));       // 残差标准化
        printf("At %d, residual is %f\n", i, residual.at<double>(i, 1));
    }
    dst = cv::Mat::zeros(4, 4, CV_64F);
    for(int i = 0; i < 4; ++i){
        double tmp = 1 / cov.at<double>(i, i);
        if(std::abs(residual.at<double>(i)) > c){
            tmp *= c / residual.at<double>(i);
        }
        c /= 80;
        dst.at<double>(i, i) = tmp;
    }
}

void Prediction::diagInv(const cv::Mat& src, cv::Mat& dst){
    for(int i = 0; i < 4; ++i){
        dst.at<double>(i, i) = 1 / src.at<double>(i, i);
    }
}

AngPre::AngPre(){
    printf("Ang pre starts to initialize.\n");
    yaw_pre.init(20);
    pitch_pre.init(40);      //要求pitch需要更加精确

    printf("Angle pre initialized.\n");
    // 录制鼠标事件验证
    #if defined(MOUSE_RECORD) || defined(RECORD_TEST)
        #ifdef MOUSE_RECORD
            rcd.record = true;
        #endif //MOUSE_RECORD

        #ifdef RECORD_TEST
            rcd.readFromFile();
            rcd.record = false;
        #endif //RECORD_TEST
    #endif  //MOUSE_RECORD
}

///=====================KALMAN集成模块======================///

bool AngPre::predict(double now_yaw, double now_pitch, cv::Point2f &res, double dt){
    res.x = now_yaw;
    res.y = now_pitch;
    ///TODO: debug，实际情况无需判断dt(delta_t当dt为正值时直接使用dt)
    double yaw_w = 0.0, yaw_b = 0.0, yaw_f = 0.0, yaw_dt = dt > 0 ? dt : -1.0;
    double pitch_w = 0.0, pitch_b = 0.0, pitch_f = 0.0, pitch_dt = dt > 0 ? dt : -1.0, mean_dt = 0.0;
    std::thread pt((&Prediction::predict), &pitch_pre, now_pitch,
            std::ref(pitch_w), std::ref(pitch_b), std::ref(pitch_f), std::ref(pitch_dt));
    yaw_pre.predict(now_yaw, yaw_w, yaw_b, yaw_f, yaw_dt);
    pt.join();      // 独立预测yaw 与 pitch 双线程
    if(yaw_dt <= 1.0 || pitch_dt <= 1.0){
        res.x = now_pitch;      // now_pitch, now_yaw 是弹道解算出来的随动角度，不加预测量的
        res.y = now_yaw;
        return false;
    }
    printf("==================================================\n");
    printf("W for yaw: %f, W for pitch: %f\n", yaw_w, pitch_w);
    printf("B for yaw: %f, B for pitch: %f\n", yaw_b, pitch_b);
    printf("F for yaw: %f, F for pitch: %f\n", yaw_f, pitch_f);
    mean_dt = (yaw_dt + pitch_dt) / 2;
    // 考虑了急动度(1/3 * f * t^3)
    double yaw_incre = 0.5 * yaw_b * mean_dt * mean_dt + 1/3 * yaw_f * pow(mean_dt, 3),
        pitch_incre = 0.5 * pitch_b * mean_dt * mean_dt + 1/3 * pitch_f * pow(mean_dt, 3),
        resx = curbedPredict(mean_dt, yaw_w, yaw_b, yaw_f, 4),
        resy = curbedPredict(mean_dt, pitch_w, pitch_b, pitch_f, 2);
    printf("Within dt:%f, the yaw_diff: %f\n", mean_dt, resx);
    printf("Within dt:%f, the pitch_diff: %f\n", mean_dt, resy);
    #ifdef MOUSE_RECORD
        rcd.input(now_yaw, now_pitch, mean_dt);         //记录鼠标事件以及鼠标事件触发间隔
    #else
        #ifdef RECORD_TEST
        rcd.input(res.x + resx, res.y + resy, mean_dt);
        #endif //RECORD_TEST
    #endif //MOUSE_RECORD
    res.x += resx;
    res.y += resy;
    return true;
}

double AngPre::curbedPredict(double dt, double w, double b, double f, double ratio){
    double linear = w * dt, incre = 0.5 * b * dt * dt + 1/3 * f * pow(dt, 3),
            incre_r = std::abs(incre / linear);
    if(incre_r > ratio){
        incre *= exp(-(incre_r - ratio));
    }
    printf("linear: %f, incre: %f\n", linear, incre);
    return (linear + incre) * exp( - std::abs(w / 15));
}

void AngPre::queueInput(double yaw, double pitch){
    ang_que.emplace_front(cv::Point2f(yaw, pitch));
}

#ifdef MOUSE_TEST
bool AngPre::testPredict(cv::Mat &src){
    bool res = true;
    #ifndef RECORD_TEST
    if(ang_que.size() == 0) {
        double now_t = cv::getTickCount();
        yaw_pre.old_time = now_t;
        pitch_pre.old_time = now_t;
        return true;
    }
    cv::Point2f input = ang_que.back();
    ang_que.pop_back();
    cv::Point2f to_draw = ang_que.front();
    double dt = -1.0;
    #else
        vec3 mevent;
        res = rcd.next(mevent);
        cv::Point2f input(mevent[0], mevent[1]);
        cv::Point2f to_draw = input;
        double dt = mevent[2];
    #endif //RECORD_TEST
    cv::Point2f predict_pos;
    if(predict(input.x, input.y, predict_pos, dt)){
        drawPoints(src, predict_pos, to_draw, true);
    }
    else{
        printf("FAILED TO PREDICT, TIME GAP TOO SMALL\n");
        drawPoints(src, predict_pos, to_draw, false);
    }
    return res;
}

void AngPre::drawPoints(cv::Mat &src, const cv::Point2f &pre, const cv::Point2f &act, bool success){
    if(success){
        cv::circle(src, pre, 4, cv::Scalar(0, 200, 200), -1);   //预测成功黄色
    }
    else{
        cv::circle(src, pre, 4, cv::Scalar(0, 0, 200), -1);     //预测失败红色
    }
    cv::circle(src, act, 2, cv::Scalar(0, 200, 0), -1);         //truth 绿色
}

#endif //MOUSE_TEST

#ifdef MOUSE_RECORD
        
#endif  //MOUSE_RECORD