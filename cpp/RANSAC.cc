#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

// 本质矩阵的求解

int main(int, char **){
    cv::Mat pic1 = cv::imread("/home/sentinel/testPics/p1.jpg");
    cv::Mat pic2 = cv::imread("/home/sentinel/testPics/p2.jpg");
    std::cout << "Pic size: " << pic1.size() << std::endl;

    cv::resize(pic1, pic1, cv::Size(1200, 900));
    cv::resize(pic2, pic2, cv::Size(1200, 900));

    cv::Ptr<cv::FeatureDetector> detect = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> describe = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // 检测角点
    std::vector<cv::KeyPoint> features1, features2;
    detect->detect(pic1, features1);
    detect->detect(pic2, features2);

    // 描述子计算
    cv::Mat des1, des2;
    describe->compute(pic1, features1, des1);
    describe->compute(pic2, features2, des2);

    // 进行粗匹配
    std::vector<cv::DMatch> matches;
    matcher->match(des1, des2, matches);

    // 这种依靠匹配阈值来判定是否为良好匹配的方法并不好
    // auto min_max = std::minmax_element(matches.begin(), matches.end(), 
    //     [](const cv::DMatch& m1, const cv::DMatch& m2){
    //         return m1.distance < m2.distance;
    //     }
    // );
    // double min_dist = min_max.first[0].distance;
    // std::vector<cv::DMatch> high_quality;
    // for (size_t i = 0; i < matches.size(); i++){
    //     if (matches[i].distance < cv::max(2 * min_dist, 50.0)){
    //         high_quality.push_back(matches[i]);
    //     }
    // }

    // 思路：使用cv::getPerpective 获取随机四点的透视变换矩阵
    // 计算透视变换矩阵之后将其他匹配点 进行透视变换（一个最优的透视变换，应该让进行可能多的匹配点对在变换后的误差小于某个阈值）
    // 需要判定是否共线
    // 计算内点个数 ... 迭代
    // 最后内点个数足够大之后，或者循环达到某个迭代次数后，退出，
    // Ceres迭代解非线性最小二乘问题（用所有内点解一个透视变换矩阵出来，求完之后对所有点进行重新分类，如果內点个数有较大增数，继续求变换矩阵）
    // 最后应该可以得到最优估计
    // 感觉好麻烦
    // 可以那Python先试着搞一下（）

    cv::Mat output;//, high;
    cv::drawMatches(pic1, features1, pic2, features2, matches, output);
    // cv::drawMatches(pic1, features1, pic2, features2, high_quality, high);
    cv::namedWindow("matched", cv::WINDOW_NORMAL);
    // cv::namedWindow("high quality", cv::WINDOW_NORMAL);
    cv::imshow("matched", output);
    // cv::imshow("high_quality", high);
    cv::waitKey(0);

    cv::destroyAllWindows();

    return 0;
}