#include "../include/NDT.hpp"

cv::Mat NDT::readAndConvert(ptsType& pts, int number) const{
    std::string path = prefix + std::to_string(number) + ".png";
    cv::Mat src = cv::imread(path, 0);
    cv::threshold(src, src, 100, 255, cv::THRESH_TOZERO);
    cv::Mat dst(cv::Size(src.cols, src.rows), CV_64FC1);
    src.convertTo(dst, CV_64FC1);
    cv::normalize(dst, dst, 1.0, 0.0, cv::NORM_INF);
    pts.resize(dst.cols * dst.rows);
    dst.forEach<double>(
        [&](const double& pix, const int* pos){
            pts[pos[1] + pos[0] * dst.cols] = std::make_pair(pix, cv::Point2d((double)pos[1] + 0.5, (double)pos[0] + 0.5));
        }
    );
    return src;
}

void NDT::readAndConvert(std::vector<double>& pts, cv::Mat& dst1, cv::Mat& dst2, int number) const{
    std::string path = prefix + std::to_string(number) + ".png";
    dst1 = cv::imread(path, 0);
    cv::threshold(dst1, dst1, 60, 255, cv::THRESH_TOZERO);
    dst2.create(cv::Size(dst1.cols, dst1.rows), CV_64FC1);
    dst1.convertTo(dst2, CV_64FC1);
    cv::normalize(dst2, dst2, 1.0, 0.0, cv::NORM_INF);
    pts.resize(dst2.cols * dst2.rows);
    dst2.forEach<double>(
        [&](const double& pix, const int* pos){
            pts[pos[1] + pos[0] * dst2.cols] = pix;
        }
    );
}

void NDT::diffusionEstimate(const cv::Mat& src1, double* top, double* ctr, double* pars) {
    int col = src1.cols, row = src1.rows;
    top[0] = double(col) / 2.0;
    top[1] = 0.0;
    ctr[0] = top[0];
    ctr[1] = double(row) / 2.0;
    pars[0] = 3.0;          // 取a = 3时，截止（10 / 255）位置近似为b + 1(.06)
    pars[1] = 2;
}

void NDT::initParamEstimate(const ptsType& pts, double* u, double* cov) const{
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
    for (const std::pair<double, cv::Point2d>& pr : pts) {
        mu += pr.first * pr.second;
        weight_sum += pr.first;
    }
    mu /= weight_sum;
    u[0] = mu.x;
    u[1] = mu.y;
    cov[0] = 0.0;
    cov[1] = 0.0;
    cov[2] = 0.0;
    for (const std::pair<double, cv::Point2d>& pr: pts) {
        cv::Point2d dpt = pr.second - mu;
        cov[0] += dpt.x * dpt.x * pr.first;
        cov[1] += dpt.y * dpt.y * pr.first;
        cov[2] += dpt.x * dpt.y * pr.first;
    }
    // 协方差也根据灰度进行加权
    cov[0] /= weight_sum;        
    cov[1] /= weight_sum;
    cov[2] /= weight_sum;
}

void NDT::lightMatching(cv::Mat& src1, cv::Mat& src2, int number, double sig_c){
    double mu[2];
    double cov[3];
    std::vector<std::pair<double, cv::Point2d> > pts;
    src1 = readAndConvert(pts, number).clone();
    src2.create(src1.rows, src1.cols, CV_8UC1);
    // std::cout << "Image loaded from '" << prefix + std::to_string(number) + ".png'\n";
    // std::cout << "Image size: " << src1.rows << ", " << src1.cols << std::endl;
    initParamEstimate(pts, mu, cov);
    double det = cov[0] * cov[1] - cov[2] * cov[2];      // 行列式
    double icov[3] = {cov[1] / det, cov[0] / det, -cov[2] / det};
    // printf("Initial params: %f, %f, %f, %f, %f\n", mu[0], mu[1], cov[0], cov[1], cov[2]);
    // printf("Size of pts: %lu\n", pts.size());
    ceres::Problem prob;
    ceres::CostFunction* cost = Error::Create(pts, sig_c);
    prob.AddResidualBlock(cost, nullptr, mu, icov);
    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = false;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
    // std::cout << summary.FullReport() << std::endl;
    // std::cout << "Translation: " << mu[0] << ", " << mu[1] << std::endl;
    Eigen::Matrix2d rot;
    det = icov[0] * icov[1] - icov[2] * icov[2];      // 行列式
    rot << icov[1] / det, -icov[2] / det, -icov[2] / det, icov[0] / det;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(rot, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector2d direct = svd.matrixU().col(0);
    // std::cout << "Main direction: " << direct(0) << ", " << direct(1) << std::endl;
    // printf("Final optimal: %f, %f, %f, %f, %f\n", mu[0], mu[1], icov[1] / det, icov[0] / det, -icov[2] / det);
    Eigen::Vector2d temp(mu[0], mu[1]);
    drawGaussian(src2, temp, rot, sig_c);
    saveToFile(src1, src2, number);
}

void NDT::lightDiffusion(cv::Mat& src1, cv::Mat& src2, int number) {
    double top[2];
    double ctr[2];
    double params[2];
    cv::Mat tmp;
    std::vector<double> values;
    readAndConvert(values, src1, tmp, number);
    src2.create(src1.rows, src1.cols, CV_8UC1);
    diffusionEstimate(src1, top, ctr, params);
    printf("Initial condition: %lf, %lf, %lf, %lf, %lf, %lf\n", top[0], top[1], ctr[0], ctr[1], params[0], params[1]);
    ceres::Problem prob;
    ceres::CostFunction* cost = ErrorTerm::Create(values, src1.cols, src1.rows);
    prob.AddResidualBlock(cost, nullptr, top, ctr, params);
    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = false;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
    // std::cout << summary.FullReport() << std::endl;
    printf("End condition: %lf, %lf, %lf, %lf, %lf, %lf\n", top[0], top[1], ctr[0], ctr[1], params[0], params[1]);
    drawDiffusion(src2, top, ctr, params);
    saveToFile(src1, src2, number);
}

void NDT::drawGaussian(cv::Mat& src, const Eigen::Vector2d& mu, const Eigen::Matrix2d& cov, double sig){
    Eigen::Matrix2d icov = cov.inverse();
    src.forEach<uchar>(
        [&](uchar& pix, const int *pos){
            Eigen::Vector2d vec((double)pos[1] - mu(0), (double)pos[0] - mu(1));
            double res = vec.transpose() * icov * vec;
            res = std::pow(res, sig);
            pix = (uchar)(exp(-res) * 255);
        }
    );
}

template <typename T>
void NDT::drawDiffusion(cv::Mat& src, const T* const _top, const T* const _ctr, const T* const _dec) const {
    int img_col = src.cols, img_row = src.rows;
    Eigen::Matrix<T, 2, 1> top(_top[0], _top[1]);
    Eigen::Matrix<T, 2, 1> ctr(_ctr[0], _ctr[1]);
    Eigen::Matrix<T, 2, 1> t2c = ctr - top;         // top -> center
    T t2c_norm = t2c.norm();                        // 法向量需要单位化
    Eigen::Matrix<T, 2, 1> bottom = ctr + t2c;
    Eigen::Matrix<T, 2, 1> normal(t2c(1) / t2c_norm, - t2c(0) / t2c_norm);
    #pragma omp parallel for num_threads(8) 
    for (int i = 0; i < img_row; i++) {
        uchar* row_data = &src.data[i * img_col];
        int base = i * img_col;
        for (int j = 0; j < img_col; j++) {
            Eigen::Matrix<T, 2, 1> t2n(T(j) - top(0), T(i) - top(1)), b2n(T(j) - bottom(0), T(i) - bottom(1));
            T t_len = t2n.norm(), b_len = b2n.norm(); 
            T prod = b_len >= t_len ? t2n.dot(t2c) : b2n.dot(- t2c);  // 更加靠近顶部 ? 内积求夹角cos : -t2c = b2c
            T dist = T(0);
            if (prod >= T(0)) {                        // 90度夹角之内
                dist = b_len >= t_len ? std::abs(t2n.dot(normal)) : std::abs(b2n.dot(normal));
            }
            else {
                dist = b_len >= t_len ? t_len : b_len;
            }
            T decay = T(1) / (std::exp(_dec[0] * (dist - _dec[1])) + T(1));     // 计算光线衰减
            row_data[j] = uchar(decay * 255);
        }
    }
}

void NDT::saveToFile(const cv::Mat& src1, const cv::Mat& src2, int number) const {
    cv::Mat rsz1, rsz2, src;
    cv::resize(src1, rsz1, cv::Size(src1.cols, src1.rows));
    cv::resize(src2, rsz2, cv::Size(src2.cols, src2.rows));
    cv::hconcat(rsz1, rsz2, src);
    std::string path = prefix + "out/" + std::to_string(number) + ".png";
    cv::imwrite(path, src);
}