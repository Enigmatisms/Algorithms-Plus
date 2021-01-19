#include <map>
#include <vector>
#include <utility>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <exception>
#include <memory>
#define varName(x) #x
#define cmpCout(x, y) std::cout << #x << " comparing with " << #y << ":" << (x > y) << ", " << (x < y) << ", " << (x == y) << "\n"

typedef std::map<int, std::string> MAP;
typedef std::unordered_map<int, std::string> UMAP;

template<typename Ty = int>
struct SelfException: public std::exception
{
    std::unique_ptr<char> what_err;             // unique_ptr本质是一个指针，使用get得到对应的初级指针
    SelfException(Ty _key):
        what_err(new char)
    {
        snprintf(what_err.get(), 48, "No such key as '%d' is found in the map.\n", _key);
    }

    const char* what() const throw() {
        return what_err.get();
    }
};

void pairTest(){
    std::vector<std::pair<int, std::string> > vec = {           // 初始化方法， const char* 显式转化为std::string
        {0, "Test pair"},
        {1, "Pair in the vector"},
        {2, "Vector containing pairs"}
    };
    for (const std::pair<int, std::string>& pr: vec){           // 两个内置的变量，first second
        std::cout << pr.first << ":" << pr.second << std::endl;
    }
    vec.push_back(
        std::make_pair(3, "New pair pushed in.")                // 相当于emplace
    );
    for (const std::pair<int, std::string>& pr: vec){
        std::cout << pr.first << ":" << pr.second << std::endl;
    }
    std::cout << "\nTest operands.\n";
    std::pair<int, double> p1(1, 2.5);
    std::pair<int, double> p2(1, 3.5);
    std::pair<int, double> p3(2, 2.5);
    std::pair<int, double> p4(2, 3.5);
    std::pair<int, double> p5(1, 1.5);
    std::pair<int, double> p6(0, 4.5);
    std::cout << "\t\t>\t<\t=\n";
    cmpCout(p1, p2);                                            // 变量比较
    cmpCout(p1, p3);
    cmpCout(p1, p4);
    cmpCout(p1, p5);
    cmpCout(p1, p6);
    cmpCout(p1, p1);
    std::cout << "Position1: " << std::get<0>(p1) << ", " << std::get<1>(p1) << std::endl;
}

void mapTestInitializing(MAP& mapping){
    
    for (const std::pair<int, std::string>& kv: mapping){           // 遍历key value对时可以使用std::pair进行
        std::cout << kv.first << ", " << kv.second << std::endl;
    }
    std::cout << "Key not found exception.\n";
    for (int i = 0; i < 4; i++){                                
        try{                                    // 不存在对应key会出现exceptions
            auto out = mapping.at(i);
            std::cout << out << std::endl;
        }
        catch(std::out_of_range)             // 自定义exception替换
        {
            std::cout << "No such ket as '" << std::to_string(i) << "' is found in the map.\n";
            // throw SelfException<int>(i);
        }
    }
    std::cout << "Insert test.\n";
    mapping[3] = std::string("Test whether this indexing is useeful.");     // 这是可以的
    mapping.insert({4, "Insert test2."});
    mapping.emplace(5, "Test emplace.");
    mapping.emplace(std::pair<int, std::string>(6, "Emplace with pair."));  // 这些都是可以的emplace方式
    // 能使用emplace就尽量emplace，原地操作
    for (const std::pair<int, std::string>& kv: mapping){                   // 遍历key value对时可以使用std::pair进行
        std::cout << kv.first << ", " << kv.second << std::endl;
    }
}

void mapTestAccess(MAP& mapping){
    for (size_t i = 0; i < mapping.size(); i++){
        std::cout << mapping.at(i) << std::endl;
        std::cout << "Using []:" << mapping[i] << std::endl; 
    }
    std::cout << "Using iterator:\n";
    for (MAP::const_iterator it = mapping.cbegin(); it != mapping.cend(); it++){
        std::cout << it->first << ", " << it->second << std::endl;
    }
    std::cout << "Using iterator to change the value.\n";
    int cnt = 0;
    for (MAP::iterator it = mapping.begin(); it != mapping.end(); it++, cnt++){
        it->second = std::to_string(cnt) + std::string(" is being processed.");
    }
    for (size_t i = 0; i < mapping.size(); i++){
        std::cout << mapping.at(i) << std::endl;
        std::cout << "Using []:" << mapping[i] << std::endl; 
    }

}

void unorderFind(const UMAP& umap, int key){
    auto it = umap.find(key);
    if (it == umap.end()){
        std::cout << "Key '" << std::to_string(key) << "' not found.\n";
    }
    else {
        std::cout << "Target found:" << it->second << std::endl;
    }
}

int main(int argc, char** argv){
    // pairTest();
    MAP mapping = {
        {0, "Test for map"},
        {1, "Map has been tested"},
        {2, "Output, test for insert."}
    };
    UMAP umap = {
        {0, "Unordered test."},
        {1, "Searching in UMAP"},
        {2, "Searhced in UMAP"}
    };
    mapTestAccess(mapping);
    for (int i = 0; i < 4; i++){
        unorderFind(umap, i);
    }
    return 0;
}