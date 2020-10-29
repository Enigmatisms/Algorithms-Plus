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

cv::Mat NDT::readAndConvert(std::vector<cv::Point2d>& pts, int number, uchar thresh){
    std::string path = prefix + std::to_string(number) + ".png";
    cv::Mat src = cv::imread(path), dst;
    cv::resize(src, src, cv::Size(4 * src.cols, 4 * src.rows));
    cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
    dst.forEach<uchar>(
        [&](uchar& pix, const int* pos){
            uchar temp = pix;
            if (temp > thresh){
                mtx.lock();
                pts.emplace_back((double)pos[1] + 0.5, (double)pos[0] + 0.5);
                mtx.unlock();
            }
        }
    );
    return src;
}

void NDT::initParamEstimate(const std::vector<cv::Point2d>& pts, double* u, double* covp) const{
    if (pts.size() < 5){
        std::cerr << "PTS size less than 5! size:" << pts.size() << std::endl;
        u[0] = 0.0;
        u[1] = 0.0;
        covp[0] = 0.0;
        covp[1] = 0.0;
        covp[2] = 0.0;
        return;
    }
    cv::Scalar mu = cv::mean(pts);
    u[0] = mu[0];
    u[1] = mu[1];
    covp[0] = 0.0;
    covp[1] = 0.0;
    covp[2] = 0.0;
    for (const cv::Point2d& pt: pts){
        double dx = pt.x - mu[0], dy = pt.y - mu[1];
        covp[0] += dx * dx * 2;
        covp[1] += dy * dy * 2;
        covp[2] += dx * dy * 2;
    }
    // 协方差计算
    covp[0] /= (double)(pts.size() - 1);        
    covp[1] /= (double)(pts.size() - 1);
    covp[2] /= (double)(pts.size() - 1);
}

void NDT::lightMatching(cv::Mat& src1, cv::Mat& src2, int number, double sig_c){
    double* mu = new double[2];
    double* cov = new double[3];
    std::vector<cv::Point2d> pts;
    src1 = readAndConvert(pts, number).clone();
    src2.create(src1.rows, src1.cols, CV_8UC1);
    std::cout << "Image loaded from '" << prefix + std::to_string(number) + ".png'\n";
    std::cout << "Image size: " << src1.rows << ", " << src1.cols << std::endl;
    initParamEstimate(pts, mu, cov);
    printf("Initial params: %f, %f, %f, %f, %f\n", mu[0], mu[1], cov[0], cov[1], cov[2]);
    printf("Size of pts: %lu\n", pts.size());
    ceres::Problem prob;
    for (const cv::Point2d& pt: pts){
        ceres::CostFunction* cost = Error::Create(pt, sig_c);
        ceres::LossFunction* huber = new ceres::HuberLoss(4);
        prob.AddResidualBlock(cost, huber, mu, cov);
        cv::circle(src1, cv::Point(pt.x, pt.y), 0, cv::Scalar(0, 0, 255), -1);
    }
    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = true;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
    std::cout << summary.FullReport() << std::endl;
    std::cout << "Translation: " << mu[0] << ", " << mu[1] << std::endl;
    Eigen::Matrix2d rot;
    rot << cov[0], cov[2], cov[2], cov[1];
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(rot, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector2d direct = svd.matrixU().col(0);
    std::cout << "Main direction: " << direct(0) << ", " << direct(1) << std::endl;
    printf("Final optimal: %f, %f, %f, %f, %f\n", mu[0], mu[1], cov[0], cov[1], cov[2]);
    Eigen::Vector2d temp(mu[0], mu[1]);
    drawGaussian(src2, temp, rot);
    delete [] mu;
    delete [] cov;
}

void NDT::drawGaussian(cv::Mat& src, const Eigen::Vector2d& mu, const Eigen::Matrix2d& cov){
    Eigen::Matrix2d icov = cov.inverse();
    src.forEach<uchar>(
        [&](uchar& pix, const int *pos){
            Eigen::Vector2d vec((double)pos[1] - mu[0], (double)pos[0] - mu[1]);
            pix = (uchar)(exp(- vec.transpose() * icov * vec) * 255);
        }
    );
}