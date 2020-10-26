#include "../include/NDT.hpp"

void NDT::initDistribute(cv::Mat& src){
    cv::Mat gauss(cv::Size(X_SIG * 2, Y_SIG * 2), CV_8UC1);
    gauss.forEach<uchar>(
        [&](uchar &pix, const int* pos) -> void{
            uchar temp = (uchar)(gauss2d(pos[1], pos[0]) * 255.0);        // 得到一张gauss分布图像
            pix = temp;
            if (temp > 200){      // 40 是一个参数
                mtx.lock();
                init_pts.emplace_back((double)pos[1] + 0.5, (double)pos[0] + 0.5);
                mtx.unlock();
            }
        }
    );
    cv::Rect pos((int)(X_SIG / 2), (int)(Y_SIG / 2), gauss.cols, gauss.rows);
    cv::cvtColor(gauss, src, cv::COLOR_GRAY2BGR);
    for (const cv::Point2d& pt: init_pts){
        cv::circle(src, cv::Point(pt.x, pt.y), 0, cv::Scalar(0, 200, 0), -1);
    }
    std::cout << "Init points size: " << init_pts.size() << std::endl;
}

void NDT::getPerturbSim(cv::Mat& src, std::vector<cv::Point2d>& pts, double& a, cv::Point2d& mov, double sig){
    a = ANGLE + rng->uniform(-0.3, 0.3);
    double x = TRANS_X + rng->uniform(-2, 10), y = TRANS_Y + rng->uniform(-2, 10);
    mov.x = x;
    mov.y = y;
    for (const cv::Point2d& pt: init_pts){
        double _x = cos(a) * pt.x - sin(a) * pt.y + x;
        double _y = sin(a) * pt.x + cos(a) * pt.y + y;
        _x += rng->gaussian(sig);
        _y += rng->gaussian(sig);
        pts.emplace_back(_x, _y);
        cv::circle(src, cv::Point(_x, _y), 0, cv::Scalar(0, 0, 255), -1);
    }
}

void NDT::minimize(const std::vector<cv::Point2d>& pts, cv::Mat& src, double init_angle){
    ceres::Problem prob;
    Eigen::Matrix2d icov = cov.inverse();
    double *param = new double[3];
    cv::Scalar start = cv::mean(pts);
    param[0] = start[0] + rng->gaussian(1);          // 模拟观测误差
    param[1] = start[1] + rng->gaussian(1);
    param[2] = init_angle + rng->gaussian(0.1); 
    for (const cv::Point2d& pt: pts){
        ceres::CostFunction* cost = ErrorTerm::Create(pt, mu, icov);
        ceres::LossFunction* huber = new ceres::HuberLoss(10);
        prob.AddResidualBlock(cost, huber, param);
    }
    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = true;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
    std::cout << summary.FullReport() << std::endl;
    double angle = param[2];
    for (const cv::Point2d& pt: pts){
        double x = cos(angle) * pt.x - sin(angle) * pt.y + param[0];
        double y = sin(angle) * pt.x + cos(angle) * pt.y + param[1];
        cv::Point p(x, y);
        cv::circle(src, p, 0, cv::Scalar(255, 0, 0), -1);
    }
    std::cout << "Translation: " << param[0] << ", " << param[1] << std::endl;
    std::cout << "Rotation angle in rads:"  << angle << std::endl;
    delete [] param;
}

void NDT::readAndConvert(std::string path, std::vector<cv::Point2d>& pts){
    cv::Mat src = cv::imread(path);
    src.forEach<uchar>(
        [&](uchar& pix, const int* pos){
            if (pix > 60){
                mtx.lock();
                pts.emplace_back((double)pos[1] + 0.5, (double)pos[0] + 0.5);
                mtx.unlock();
            }
        }
    );
}