#include <opencv2/opencv.hpp>

/**
 * @brief 判断本帧是否是低曝光/ @author Dingger
 * @return 如果本帧低曝光，返回真
 */
bool isLowExposure(cv::Mat &src) {
	cv::Scalar mean_values;
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    int row_sampling = 16;
    int col_sampling = 16;
    int sampling_sum = 0;
    for(int i=0;i<row_sampling;++i){
        for(int j=0;j<col_sampling;++j){
            int pix_value = gray.at<uchar>(gray.rows*0.1+i*gray.rows*0.8/row_sampling, gray.cols*0.1+j*gray.cols*0.8/col_sampling);
            if(pix_value>30) ++sampling_sum;
            // cv::Point2f smp = cv::Point2f(gray.cols*0.1+i*gray.cols*0.8/col_sampling, gray.rows*0.1+j*gray.rows*0.8/row_sampling);
            // cv::circle(src, smp, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
    if(sampling_sum < row_sampling*col_sampling*0.3)
	    return true;
    else
        return false;
}
