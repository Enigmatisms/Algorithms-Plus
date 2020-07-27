#ifndef STACK_LEARN
#define STACK_LEARN

#include <stack>
#include <vector>
#include <iostream>
#include <cstdio>

enum OPS{
    ADD,        // 加
    SUB,        // 减
    MUL,        // 乘
    DIV,        // 除
    LEFT,       // 左括号
    RIGHT,      // 右括号
    END         // 终止符
};

// > 为 1, < 为 -1, = 为 0, 2 为错误, a > b 标示 a 优先级更大
const int priority[7][7] = {
    {1, 1, -1, -1, -1, 1, 1},
    {1, 1, -1, -1, -1, 1, 1},
    {1, 1, 1, 1, -1, 1, 1},
    {1, 1, 1, 1, -1, 1, 1},
    {-1, -1, -1, -1, -1, 0, 2},
    {1, 1, 1, 1, 2, 1, 1},
    {-1, -1, -1, -1, -1, 2, 0}
};

class Learn{
// constructor / de-con
public:
    Learn();
    ~Learn();
//func
public:

private:

//var
public:

private:
    std::stack<OPS> oprt;
    std::stack<float> oprd;

};

#endif  //STACK_LEARN