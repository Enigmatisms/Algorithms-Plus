#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <string.h>
#include <opencv2/core.hpp>
// 测试一下引用和指针

int main(){
    std::vector<int> vec = {1, 2, 3, 4, 5, 6, 7};
    int* now = &vec.front();
    std::cout << "Address for ptr: " << now << std::endl;
    std::cout << "Address for vec[0]: " << &vec[0] << std::endl;
    std::stack<int> st;
    st.push(1);
    st.push(2);
    st.push(3);
    st.push(4);
    st.push(5);
    int* stp = &st.top();
    std::cout << "Address for st ptr: " << stp << std::endl;
    std::cout << "Address for st top: " << &st.top() << std::endl;
    std::cout << "Top: " << st.top() << std::endl;
    std::cout << "Top ptr: " << *stp << std::endl;
    st.pop();
    // 发现：stack pop 之后,指向top的指针 stp 未变（指向同一个地址）

    std::cout << "Top: " << st.top() << std::endl;
    std::cout << "Top ptr: " << *stp << std::endl;
    std::cout << "Address for st ptr: " << stp << std::endl;
    std::cout << "Address for st top: " << &st.top() << std::endl;
    // 此后 stack emplace 会导致原来的地址填上新的数据

    st.emplace(100);
    std::cout << "Top: " << st.top() << std::endl;
    std::cout << "Top ptr: " << *stp << std::endl;
    std::cout << "Address for st ptr after emplace: " << stp << std::endl;
    std::cout << "Address for st top after emplace: " << &st.top() << std::endl;

    // 指针指向的内存位置（地址）一直不变，并且stack每一层使用的区域也不变
    // 比如stack 内有四个元素，他们的地址为（连续的A, B, C, D）
    // pop 栈顶， 三个元素地址为 (A, B, C) 而指向stack.top()的指针一直指向D不变
    // 再次emplace, 由于栈使用连续的内存，D又会被重新使用，则指针指向地址的值为emplace值
    // 这也就是在stack_dfs中，使用指针出现问题的原因
    st.emplace(120);
    std::cout << "Top: " << st.top() << std::endl;
    std::cout << "Top ptr: " << *stp << std::endl;
    std::cout << "Address for st ptr after emplace: " << stp << std::endl;
    std::cout << "Address for st top after emplace: " << &st.top() << std::endl;

    return 0;
}