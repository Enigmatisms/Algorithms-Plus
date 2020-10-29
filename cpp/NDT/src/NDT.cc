#include "../include/NDT.hpp"

cv::Mat NDT::readAndConvert(std::vector<cv::Point3d>& pts, int number, uchar thresh){
    std::string path = prefix + std::to_string(number) + ".png";
    cv::Mat src = cv::imread(path), dst;
    cv::resize(src, src, cv::Size(4 * src.cols, 4 * src.rows));
    cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
    dst.forEach<uchar>(
        [&](uchar& pix, const int* pos){
            uchar temp = pix;
            // if (temp > thresh){
                mtx.lock();
                pts.emplace_back((double)pos[1] + 0.5, (double)pos[0] + 0.5, (double)temp / 255.0);
                mtx.unlock();
            // }
        }
    );
    return src;
}

void NDT::initParamEstimate(const std::vector<cv::Point3d>& pts, double* u, double* cov) const{
    if (pts.size() < 5){
        std::cerr << "PTS size less than 5! size:" << pts.size() << std::endl;
        u[0] = 0.0;
        u[1] = 0.0;
        cov[0] = 0.0;
        cov[1] = 0.0;
        cov[2] = 0.0;
        return;
    }
    cv::Point2d mu(0, 0);
    double weight_sum = 0;
    for (const cv::Point3d& pt : pts) {
        mu += cv::Point2d(pt.x, pt.y) * pt.z;
        weight_sum += pt.z;
    }
    mu /= weight_sum;
    u[0] = mu.x;
    u[1] = mu.y;
    cov[0] = 0.0;
    cov[1] = 0.0;
    cov[2] = 0.0;
    for (const cv::Point3d& pt: pts) {
        cv::Point2d dpt(pt.x - mu.x, pt.y - mu.y);
        cov[0] += dpt.x * dpt.x;
        cov[1] += dpt.y * dpt.y;
        cov[2] += dpt.x * dpt.y;
    }
    // 协方差计算
    cov[0] /= (double)(pts.size() - 1);        
    cov[1] /= (double)(pts.size() - 1);
    cov[2] /= (double)(pts.size() - 1);
}

void NDT::lightMatching(cv::Mat& src1, cv::Mat& src2, int number, double sig_c){
    double mu[2];
    double cov[3];
    std::vector<cv::Point3d> pts;
    src1 = readAndConvert(pts, number, 0).clone();
    src2.create(src1.rows, src1.cols, CV_8UC1);
    std::cout << "Image loaded from '" << prefix + std::to_string(number) + ".png'\n";
    std::cout << "Image size: " << src1.rows << ", " << src1.cols << std::endl;
    initParamEstimate(pts, mu, cov);
    double det = cov[0] * cov[1] - cov[2] * cov[2];      // 行列式
    double icov[3] = {cov[1] / det, cov[0] / det, -cov[2] / det};
    printf("Initial params: %f, %f, %f, %f, %f\n", mu[0], mu[1], cov[0], cov[1], cov[2]);
    printf("Size of pts: %lu\n", pts.size());
    ceres::Problem prob;
    ceres::CostFunction* cost = Error::Create(pts, sig_c);
    prob.AddResidualBlock(cost, nullptr, mu, icov);
    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = true;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
    std::cout << summary.FullReport() << std::endl;
    std::cout << "Translation: " << mu[0] << ", " << mu[1] << std::endl;
    Eigen::Matrix2d rot;
    det = icov[0] * icov[1] - icov[2] * icov[2];      // 行列式
    rot << icov[1] / det, -icov[2] / det, -icov[2] / det, icov[0] / det;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(rot, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector2d direct = svd.matrixU().col(0);
    std::cout << "Main direction: " << direct(0) << ", " << direct(1) << std::endl;
    printf("Final optimal: %f, %f, %f, %f, %f\n", mu[0], mu[1], icov[1] / det, icov[0] / det, -icov[2] / det);
    Eigen::Vector2d temp(mu[0], mu[1]);
    drawGaussian(src2, temp, rot);
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