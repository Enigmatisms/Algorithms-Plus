#ifndef __PID_CTRL__
#define __PID_CTRL__

#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <cmath>
// PID 以及变结构 PID的实现
// 一维PID

template <class Ty>
class PIDModule {
public:
    PIDModule(std::string path = "../output.txt") {
        reset();
        kp = 0;
        ki = 0;
        kd = 0;
        ap = 0;
        bp = 0;
        cp = 0;
        ai = 0;
        ci = 0;
        ad = 0;
        file = new std::ofstream(path, std::ios::out);
        *file << "[";
    }
    ~PIDModule() {
        std::cout << "PIDModule starts to release.\n";
        *file << "]";
        delete file;
        file = nullptr;
    }
public:
    // 朴素PID参数初始设置
    void naiveSetup(Ty _p, Ty _i, Ty _d);
    
    // 变结构PID参数初始设置
    void variableSetup(Ty* params);
    
    // 朴素增量式PID
    Ty naivePID(Ty now, Ty ctrl);

    // 变结构PID
    Ty variablePID(Ty now, Ty ctrl);

    // 写入文件
    void writeLineInFile(Ty t, Ty y_exp, Ty y_act, bool final = false);

    // 重置PID
    void reset();
private:
    Ty accum;        // 累加值（可能会越来越大）
    Ty old_error;     // 历史误差
    Ty kp;            // 朴素PID参数
    Ty ki;
    Ty kd;
    Ty ap;             // 变结构PID比例参数
    Ty bp;
    Ty cp;
    Ty ai;            // 变结构PID积分参数
    Ty ad;
    Ty ci;
    std::ofstream* file;
};

template <class Ty>
Ty PIDModule<Ty>::naivePID(Ty now, Ty ctrl) {
    // err 为当前目标控制输出与当前量的偏差
    Ty err = ctrl - now;
    // delta_error 为误差差分
    Ty delta_error = err - old_error;
    // 累计误差
    accum += err;
    old_error = err;
    // 返回增量（当前被控量的增量结果）
    return kp * err + ki * accum + kd * delta_error;
}

template <class Ty>
Ty PIDModule<Ty>::variablePID(Ty now, Ty ctrl) {
    Ty err = ctrl - now;
    Ty delta_error = err - old_error;
    accum += err;
    old_error = err;
    Ty res = (ap + bp * (1 - exp(-cp * std::abs(err)))) * err
        + ai * exp(-ci * std::abs(err)) * accum
        + ad * delta_error;
    return res;
}

template <class Ty>
void PIDModule<Ty>::reset(){
    accum = (Ty)0;
    old_error = (Ty)0;
}

template <class Ty>
void PIDModule<Ty>::naiveSetup(Ty _p, Ty _i, Ty _d){
    kp = _p;
    ki = _i;
    kd = _d;
}

template <class Ty>
void PIDModule<Ty>::variableSetup(Ty* params){
    ap = params[0];
    bp = params[1];
    cp = params[2];
    ai = params[3];
    ci = params[4];
    ad = params[5];
}

template <class Ty>
void PIDModule<Ty>::writeLineInFile(Ty t, Ty y_exp, Ty y_act, bool final){
    *file << "[" << t << ", " << y_exp << ", " << y_act << "]";
    if (!final){
        *file << ",\n";
    }
}

// 正弦波控制
double sinCtrl(double t, double A = 10.0, double w = 0.5, double C = 10.0){
    return sin(w * t) * A + C;
}

// 方波发生
double rectWave(double t){
    if (t < 20){
        return 0;
    }
    else if (t < 40){
        return 20;
    }
    else if (t < 60){
        return 0;
    }
    else if (t < 80){
        return 20;
    }
    return 0;
}

// 幅度变化的正弦波
double ampSin(double t){
    return 30 * sin(0.05 * t) * sin(0.5 * t);
}

#endif // __PID_CTRL__



