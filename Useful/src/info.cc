#include <algorithm>
#include "../include/distance/LightMatch.hpp"
#include "../include/distance/ArmorPlate.hpp"

bool isLowExposure(cv::Mat &src);

int main(int argc, char ** argv){
    int delay_time = 100;
    double dist = atof(argv[1]),
        angle = atof(argv[2]);
    cv::VideoCapture cap;
    std::string video = std::string("/home/sentinel/standard/cv_d")
        + std::string(argv[1]) + std::string("a") + std::string(argv[2]) + std::string(".avi");
    cap.open(video);
    std::cout << video << " is now being processed with " <<
            (int)cap.get(cv::CAP_PROP_FRAME_COUNT) << " frames."<< std::endl;

    LightMatch match;
    ArmorPlate amp(dist, angle);
    cv::Mat src;
    

    match.setEnemyColor(true);          // 敌人蓝色

    if(!cap.isOpened()){
        std::cout << "Can not open: " << video << std::endl;
        return 0;
    }
    cv::namedWindow("disp", cv::WINDOW_KEEPRATIO);
    std::vector<aim_deps::Armor> tar_list;
    cap.read(src);
    while(!src.empty()){
        if(isLowExposure(src)){                   //通道更换(enemy_blue)
            match.saveImg(src);
    	    match.findPossible();
    	    amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条
            amp.postProcess(tar_list);
        }
        amp.drawArmorPlates(src, tar_list, 0);
        cv::imshow("disp", src);
        char key = cv::waitKey(delay_time);
        if(key == ' '){
            cv::waitKey(0);
        }
        else if(key == 27){
            break;
        }
        else if(key == 'e'){
            delay_time = 100 ? 1000 : 100;
        }
        cap.read(src);
    }
    cap.release();

    return 0;
}