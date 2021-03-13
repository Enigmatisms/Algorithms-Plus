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

void NDT::readAndConvert(std::vector<double>& pts, cv::Mat& dst, int number) const{
    std::string path = prefix + std::to_string(number) + ".png";
    dst = cv::imread(path, 0);
    cv::threshold(dst, dst, 60, 255, cv::THRESH_TOZERO);
    cv::Mat dst2(cv::Size(dst.cols, dst.rows), CV_64FC1);
    dst.convertTo(dst2, CV_64FC1);
    cv::normalize(dst2, dst2, 1.0, 0.0, cv::NORM_INF);
    pts.resize(dst2.cols * dst2.rows);
    dst2.forEach<double>(
        [&](const double& pix, const int* pos){
            pts[pos[1] + pos[0] * dst2.cols] = pix;
        }
    );
}

inline float getPointDist(const cv::Point2f &p1, const cv::Point2f &p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

void getMidPoints(const cv::RotatedRect &rect, cv::Point2f &p1, cv::Point2f &p2){
    cv::Point2f tmp_p1, tmp_p2, corners[4];                                     //找出角点
    rect.points(corners);
    float d1 = getPointDist(corners[0], corners[1]);            //0/1点距离的平方
	float d2 = getPointDist(corners[1], corners[2]);            //1/2点距离的平方
	int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
    tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;			    //获得旋转矩形两条短边上的中点
	tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
    if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
        p2 = (tmp_p1 + tmp_p2) / 2;    
        p1 = tmp_p2;
    }
    else{                                                       //必须是p1是处于上方的点，p2处于下方（y轴更大）
        p1 = tmp_p1;
        p2 = (tmp_p1 + tmp_p2) / 2;
    }
}

void NDT::betterInitialize(const cv::Mat& src, double* _top, double* _ctr) const {
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat dst;
    cv::threshold(src, dst, 1, 255, cv::THRESH_BINARY);
    cv::findContours(dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    const std::vector<cv::Point>& pts = *std::max_element(contours.begin(), contours.end(), 
        [&](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return c1.size() < c2.size();
        }
    );
    cv::RotatedRect rect = cv::minAreaRect(pts);
    cv::Point2f tp, mp;
    getMidPoints(rect, tp, mp);
    _top[0] = tp.x;
    _top[1] = tp.y;
    _ctr[0] = mp.x;
    _ctr[1] = mp.y;
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
    opts.minimizer_type = ceres::LINE_SEARCH;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-5;
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

void NDT::lightDiffusion(cv::Mat& src1, cv::Mat& src2, int number, double radius) {
    double top[2];
    double ctr[3];
    std::vector<double> values;
    readAndConvert(values, src1, number);
    src2.create(src1.rows, src1.cols, CV_8UC1);

    betterInitialize(src1, top, ctr);
    ctr[2] = 0.0;

    printf("Initial condition: %lf, %lf, %lf, %lf, %lf, %lf\n", top[0], top[1], ctr[0], ctr[1], radius + ctr[2]);
    ceres::Problem prob;
    ceres::CostFunction* cost = ErrorTerm::Create(values, src1.cols, src1.rows, radius);
    prob.AddResidualBlock(cost, nullptr, top, ctr);
    ceres::Solver::Options opts;
    opts.minimizer_type = ceres::LINE_SEARCH;
    opts.line_search_direction_type = ceres::LBFGS;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.line_search_type = ceres::WOLFE;
    opts.minimizer_progress_to_stdout = true;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
    std::cout << summary.FullReport() << std::endl;
    printf("End condition: %lf, %lf, %lf, %lf, %lf\n", top[0], top[1], ctr[0], ctr[1], radius + ctr[2]);
    drawDiffusion(src2, top, ctr, radius + ctr[2]);
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
void NDT::drawDiffusion(cv::Mat& src, const T* const _top, const T* const _ctr, T radius) const {
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
            T decay = T(1) / (std::exp(T(_A) * (dist - radius)) + T(1));     // 计算光线衰减
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