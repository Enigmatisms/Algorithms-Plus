#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <string.h>

// DFS BFS 等非递归算法的实现 --- 数据结构与算法复习

struct Point{
    int id = 0;
    int visited = false;
    Point(){;}
    Point(const Point& pt){
        this->id = pt.id;
        this->visited = pt.visited;
    }
};

std::stack<Point> st;
std::queue<Point> que;

void DFS(std::vector<Point> &pts, int** adjs){
    std::cout << "DFS:\n";
    int now_id = 0;
    for (size_t i = 0; i < pts.size(); i++){
        if (pts[i].visited == true){
            continue;
        }
        st.emplace(pts[i]);
        while (!st.empty()){
            now_id = st.top().id;
            std::cout << now_id;
            pts[now_id].visited = true;
            st.pop();                   // 取完之后可以立即pop栈顶，在此后，马上将会将其所有的邻接且未访问点加入
            for (size_t j = 0; j < pts.size(); j++){
                if (pts[j].visited == false && adjs[now_id][j] > 0){    // 未被访问并且存在边
                    st.emplace(pts[j]);
                }
            }
            if (st.size() > 0){
                std::cout << "-->";
            }
        }
        std::cout << std::endl;
    }
}

void DFSError(std::vector<Point> &pts, int** adjs){
    std::cout << "False DFS:\n";
    Point *now;
    for (size_t i = 0; i < pts.size(); i++){
        if (pts[i].visited == true){
            continue;
        }
        st.emplace(pts[i]);
        while (!st.empty()){
            now = &st.top();
            std::cout << now->id;
            pts[now->id].visited = true;
            st.pop();                   // 取完之后可以立即pop栈顶，在此后，马上将会将其所有的邻接且未访问点加入
            for (size_t j = 0; j < pts.size(); j++){
                if (pts[j].visited == false && adjs[now->id][j] > 0){    // 未被访问并且存在边
                    st.emplace(pts[j]);                                  // 这会导致指针的值改变，因为指针指向的地址不变，stack使用连续内存
                }
            }
            if (st.size() > 0){
                std::cout << "-->";
            }
        }
        std::cout << std::endl;
    }
}

void BFS(std::vector<Point> &pts, int** adjs){
    std::cout << "BFS:\n";
    int now_id = 0;
    for (const Point &pt: pts){
        if (pt.visited == true){
            continue;
        }
        que.emplace(pt);
        while (que.empty() == false){
            now_id = que.front().id;
            pts[now_id].visited = true;
            que.pop();
            std::cout << now_id;
            for (size_t i = 0; i < pts.size(); i++){
                if (pts[i].visited == false && adjs[now_id][i]){
                    que.emplace(pts[i]);
                }
            }
            if (que.size() > 0){
                std::cout << "-->";
            }
        }
        std::cout << std::endl;
    }
}

void addPath(int x, int y, int** adjs){
    adjs[x][y] = 1;
    adjs[y][x] = 1;
}

int main(){
    size_t node_num = 10;
    int** adjs = new int *[node_num];
    for (size_t i = 0; i < node_num; i++){
        adjs[i] = new int [node_num];
    }

    addPath(0, 2, adjs);
    addPath(0, 3, adjs);
    addPath(2, 6, adjs);
    addPath(3, 7, adjs);
    addPath(3, 8, adjs);
    addPath(7, 9, adjs);
    addPath(1, 4, adjs);
    addPath(1, 5, adjs);
    std::vector<Point> pts(node_num);
    std::vector<Point> pts2(node_num);
    std::vector<Point> pts3(node_num);
    for (int i = 0; i < node_num; i++){
        pts[i].id = i;
        pts2[i].id = i;
        pts3[i].id = i;
        pts[i].visited = false;
        pts2[i].visited = false;
        pts3[i].visited = false;
    }


    for (size_t i = 0; i < node_num; i++){
        for (size_t j = 0; j < node_num; j++){
            std::cout << adjs[i][j] << ", ";
        }
        std::cout << std::endl;
    }
   
    DFS(pts, adjs);
    BFS(pts2, adjs);
    DFSError(pts3, adjs);

    for (size_t i = 0; i < node_num; i++){
        delete [] adjs[i];
    }
    delete [] adjs;
    std::cout << "Memory freed.\n";
    return 0;
}

// C++ 基础不牢
// void BFS(std::vector<Point> &pts, int** adjs){
//     std::cout << "BFS:\n";
//     Point now = pts.front();
//     for (Point &pt: pts){
//         if (pt.visited == false){
//             que.emplace(pt);
//         }
//         while (que.empty() == false){
//             now = que.back();
//             pts[now.id].visited = true;
//             que.pop();
//             std::cout << now.id << "-->";
//             for (size_t i = 0; i < pts.size(); i++){
//                 if (pts[i].visited == false && adjs[now.id][i]){
//                     que.emplace(pts[i]);
//                 }
//             }
//         }
//         std::cout << std::endl;
//     }
// }
// 以上代码跑不出来，个人猜测是引用的问题，改成指针后完全可行


// 这种写法也会导致问题，既然找到了那么多漏洞，需要搞清楚，为什么？
// 个人觉得，stack在pop时，now指针的指向可能发生了改变，没有验证过
// 因为我发现，只要记录（按值）now_id就能跑通
// 问题挺多的，在9.1日之前解决