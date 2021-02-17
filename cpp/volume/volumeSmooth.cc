#include "include/Volume.hpp"

const std::string win_name = "disp";

int main(int argc, char* argv[]){
    int number = 1;
    if (argc >= 2){
        number = atoi(argv[1]);
    }
    Volume vol(number);
    cv::namedWindow("disp");
    while (true){
        char key = cv::waitKey(0);
        if (key == ' '){
            vol.reset();
            vol.generateMap(number);
        }
        else if (key == 'w')
            vol.moveSmooth(UP, win_name);
        else if (key == 'a')
            vol.moveSmooth(LEFT, win_name);
        else if (key == 's')
            vol.moveSmooth(DOWN, win_name);
        else if (key == 'd')
            vol.moveSmooth(RIGHT, win_name);
        else if (key == 27){
            break;
        }
        else if (key == 'e'){
            vol.moveSmooth(IDLE, win_name);
        }
    }
    cv::destroyAllWindows();
    return 0;
}