#include "include/Volume.hpp"

int main(){
    Volume vol(150);
    std::cout << "Volume created.\n";
    while (true){
        vol.getOutSideEdgeByBlock();
        vol.debugDisplay();
        char key = cv::waitKey(0);
        if (key == ' '){
            vol.reset();
            vol.generateMap(150);
        }
        else if (key == 'w') {
            vol.move(UP);
            vol.reset();
        }
        else if (key == 'a') {
            vol.move(LEFT);
            vol.reset();
        }
        else if (key == 's') {
            vol.move(DOWN);
            vol.reset();
        }
        else if (key == 'd') {
            vol.move(RIGHT);
            vol.reset();
        }
        else if (key == 27){
            break;
        }
    }
    cv::destroyAllWindows();
    return 0;
}