#include "PID.hpp"
// PID 算法实现极其简单

int main(){
    double t = 0, now_ctrl = 0, state = 0;
    PIDModule<double>* naive = new PIDModule<double>;
    naive->naiveSetup(0.8, 0.2, 0.1);
    std::cout << "Preparing to start pid module...\n";
    while (t < 110){
        now_ctrl = ampSin(t);
        state += naive->naivePID(state, now_ctrl);
        naive->writeLineInFile(t, now_ctrl, state);
        t += 0.1;
    }
    now_ctrl = ampSin(t);
    state += naive->naivePID(state, now_ctrl);
    naive->writeLineInFile(t, now_ctrl, state, true);
    delete naive;
    std::cout << "Output completed.\n";

    PIDModule<double>* var = new PIDModule<double>("../output2.txt");
    double params[6] = {1.0, 0.4, 0.3, 0.2, 0.1, 0.15};
    var->variableSetup(params);
    t = 0, now_ctrl = 0, state = 0;
    std::cout << "Preparing to start variable structure pid module...\n";
    while (t < 110){
        now_ctrl = ampSin(t);
        state += var->variablePID(state, now_ctrl);
        var->writeLineInFile(t, now_ctrl, state);
        t += 0.1;
    }
    now_ctrl = ampSin(t);
    state += var->variablePID(state, now_ctrl);
    var->writeLineInFile(t, now_ctrl, state, true);
    delete var;
    std::cout << "Output completed.\n";
    return 0;
}