#include "src/Record.hpp"

int main(int, char **){

    Record<vec3> rcd;
    rcd.record = true;
    int cnt = 0;
    for (int i = 0; i < 5; ++i){
        for(int j = 0; j < 25; ++j){
            if(cnt % 2){
                rcd.input(110, 110, 13);
                rcd.input(108, 108, 13);
                rcd.input(110, 110, 13);
                rcd.input(112, 112, 13);
            }
            else{
                rcd.input(50, 50, 13);
                rcd.input(48, 48, 13);
                rcd.input(50, 50, 13);
                rcd.input(52, 52, 13);
            }
        }
        ++cnt;
    }
    rcd.writeStrInFile("/home/sentinel/self-targeting/py_standard.txt");
    printf("Rect wave input completed.\n");
    return 0;
}