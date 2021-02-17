#include "include/Volume.hpp"

int main(int argc, char* argv[]){
    int number = 1;
    if (argc >= 2){
        number = atoi(argv[1]);
    }
    Volume vol(number);
    vol.getOutSideEdgeByBlock();
    std::cout << "Volume created.\n";
    bool render_flag = false;
    while (true){
        // vol.debugDisplay(render_flag);
        char key = cv::waitKey(0);
        if (key == ' '){
            vol.reset();
            vol.generateMap(number);
            vol.getOutSideEdgeByBlock();
        }
        else if (key == 'w') {
            vol.moveStep(UP);
            vol.reset();
            vol.getOutSideEdgeByBlock();
        }
        else if (key == 'a') {
            vol.moveStep(LEFT);
            vol.reset();
            vol.getOutSideEdgeByBlock();
        }
        else if (key == 's') {
            vol.moveStep(DOWN);
            vol.reset();
            vol.getOutSideEdgeByBlock();
        }
        else if (key == 'd') {
            vol.moveStep(RIGHT);
            vol.reset();
            vol.getOutSideEdgeByBlock();
        }
        else if (key == 27){
            break;
        }
        else if (key == 'e'){
            if (vol.edges.empty() == false){
                vol.edges.top()->setValid(false);
                vol.edges.pop();
            }
        }
        else if (key == 'r' && render_flag == false){
            vol.edgeTranverse();
        }
        else if (key == 'f'){
            render_flag = !render_flag;
        }
    }
    cv::destroyAllWindows();
    return 0;
}