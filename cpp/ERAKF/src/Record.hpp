#ifndef RECORD_HPP
#define RECORD_HPP

#include <fstream>
#include <iostream>
#include <vector>

/// @brief 测试用的简易三维向量
struct vec3{
    double x;
    double y;
    double z;
    vec3(double _x, double _y, double _z){
        x = _x;
        y = _y;
        z = _z;
    }

    /// @overload
    vec3(){
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    double& operator[](int i){
        switch(i){
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw "IndexError: index not in range(0, 3)";
        }
    }
};


template<class Ty = vec3>
class Record{
public:
    Record(bool debug = false);
    ~Record();
public:
    /// 需要两手准备，我不会用C++ 画曲线，只能使用python,也就是要写一个可以用json读取的文件
    bool writeBinInFile(std::string path = "/home/sentinel/self-targeting/bin.txt");

    /// @brief 给json 读写的纯字符串输出
    bool writeStrInFile(std::string path = "/home/sentinel/self-targeting/pyf.txt");        

    bool readFromFile(std::string path = "/home/sentinel/self-targeting/bin.txt");        // 读取数据
    void display();                                                   // 显示数据 (debug)
    
    /// @brief 输入数据, x, y, z (pitch, yaw, delta_t)
    void input(double x, double y, double z);

    void reset();

    //输出contain中的下一个
    bool next(Ty &res){
        res = storage[cnt];
        ++cnt;
        if(cnt < storage.size()) return true;
        else return false;
    }
public:
    bool record;
    std::vector<Ty> contain;        //输出存放容器
    std::vector<Ty> storage;        //读取存放容器
private:
    std::vector<Ty> test_arr;
    int verify;
    int cnt;
};

template<class Ty>
Record<Ty>::Record(bool debug){
    contain.clear();
    storage.clear();
    if(debug == true){
        test_arr.resize(10);
        for(int i = 0; i < 10; ++i){
            for(int j = 0; j < 3; ++j){
                test_arr[i][j] = i * 10.0 + j;
            }
        }
    }
    printf("Record initialized.\n");
    verify = 0xffffffff;        // 校验码
    record = true;
    cnt = 0;
}

template<class Ty>
Record<Ty>::~Record(){
    if(record){                     //如果进行录制，才会写入二进制文件
        if(!writeBinInFile()){
            printf("Data unable to be saved in text.txt.\n");
        }
    }
}

template<class Ty>
void Record<Ty>::reset(){
    contain.clear();
    storage.clear();
}


template<class Ty>
bool Record<Ty>::writeBinInFile(std::string path){
    int total_size = contain.size() * sizeof(Ty);
    int length = contain.size();
    std::ofstream ofile(path.c_str(), std::ios::binary);
    if(!ofile.is_open()){
        std::cout << "Failed to open file " << path << std::endl;
        return false;
    }
    ofile.write((char *)&verify, sizeof(int));
    ofile.write((char *)&length, sizeof(int));
    ofile.write((char *)&total_size, sizeof(int));
    ofile.write((char *)&contain[0], total_size);
    ofile.write((char *)&verify, sizeof(int));
    
    ofile.close();
    return true;
}

template<class Ty>
bool Record<Ty>::writeStrInFile(std::string path){      // 输出一个python 二维list
    std::ofstream of(path.c_str(), std::ios::trunc);
    if(!of.is_open()){
        std::cout << "Failed to open from path:" << path << std::endl;
        return false;
    }
    of << "[";
    size_t i = 0;
    for(; i < contain.size() - 1; ++i){
        of << "[" << contain[i][0] << "," << contain[i][1]
           << "]," << std::endl; 
    }
    of << "[" << contain[i][0] << "," << contain[i][1]
           << "]]";
    return true;
}

template<class Ty>
bool Record<Ty>::readFromFile(std::string path){
    int len, total_sz, verifier;
    std::ifstream ifile(path.c_str(), std::ios::binary);
    ifile.read((char *)&verifier, sizeof(int));
    if(verifier != verify){
        std::cout << "Verification failed at the beginning of the file. Unknown format." << std::endl;
        return false;
    } 
    ifile.read((char *)&len, sizeof(int));
    storage.resize(len);
    ifile.read((char *)&total_sz, sizeof(int));
    ifile.read((char *)&storage[0], total_sz);
    ifile.read((char *)&verifier, sizeof(int));
    if(verifier != verify){
        std::cout << "Verification failed at the end of the file. Unknown format." << std::endl;
        return false;
    }
    return true;
}

template<class Ty>
void Record<Ty>::display(){
    for(size_t i = 0; i < storage.size(); ++i){
        printf("Storage[%lu]: %lf, %lf, %lf\n", i, storage[i][0], storage[i][1], storage[i][2]);
    }
    printf("\n");
    for(size_t i = 0; i < test_arr.size(); ++i){
        printf("Origin[%lu]: %lf, %lf, %lf\n", i, test_arr[i][0], test_arr[i][1], test_arr[i][2]);
    }
}

template<class Ty>
void Record<Ty>::input(double x, double y, double z){
    contain.emplace_back(Ty(x, y, z));      //输出容器内加入
}

#endif  //RECORD_HPP  
