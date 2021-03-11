#include <vector>
#include <memory>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>

// boost::bind 配合占位符进行partial操作
// 注意，如果被bind的目标是引用，需要使用boost::ref进行引用转化，常引用避免复制可以使用cref

// boost::function 用于生成函数指针 用于回调函数构建
// 在boost::function指向类成员时，第一个参数需要传this或者相应类指针

// STL库需要处理：
    // <algorithm> sort 以及部分 sort, for_each, transform(可能是functional中的函数)
    // <memory> 三种ptr
    // <boost> smartptr 与 multithread

template<typename Ty>
void vecCopy(const std::vector<Ty>& vec, std::vector<Ty>& dst, int status){
    dst.assign(vec.begin(), vec.end());
    printf("Status = %d\n", status);
}

void refTest(int a, int b, int& c, int& d){
    c = a + b;
    d = a - b;
}

int main(){
    int res = 3;
    std::vector<int> vec1;

    for (int i = 0; i < 10; i++){
        vec1.emplace_back(i);
    }
    std::vector<int> vec2;
    std::vector<int> vec3;
    std::vector<int> vec4;
    auto partial = boost::bind(vecCopy<int>, boost::cref(vec1), _1, res);

    partial(vec2);
    printf("Vec2:");
    for (size_t i = 0; i < vec2.size(); i++){
        printf(", %d" + !i, vec2[i]);
    }
    printf("\n");

    partial(vec3);
    printf("Vec3:");
    for (size_t i = 0; i < vec3.size(); i++){
        printf(", %d" + !i, vec3[i]);
    }
    printf("\n");

    boost::function<void (const std::vector<int>& vec, std::vector<int>& dst, int status)> fptr = vecCopy<int>;
    fptr(vec2, vec4, 5);
    printf("Vec4:");
    for (size_t i = 0; i < vec4.size(); i++){
        printf(", %d" + !i, vec4[i]);
    }
    printf("\n");

    int aa, bb, cc = 0, dd = 0;

    // 把下一行的boost::ref删除将会造成问题
    auto func2 = boost::bind(refTest, _1, _2, boost::ref(cc), boost::ref(dd));
    aa = 3;
    bb = 2;
    func2(aa, bb);
    printf("%d + %d = %d, %d - %d = %d\n", aa, bb, cc, aa, bb, dd);
    aa = 4;
    func2(aa, bb);
    printf("%d + %d = %d, %d - %d = %d\n", aa, bb, cc, aa, bb, dd);
    return 0;
}