#ifndef DIJ_HPP
#define DIJ_HPP
#include <iostream>
#include <vector>
#include <map>
#define INF 0xffff

struct Vex{
    int prev;           //前驱
    int next;           //后继
    int cost;           //本结点当前cost
    std::string name;   //节点名
    Vex(){
        prev = 0;
        next = 0;
        cost = INF;
    }
}vex;

struct graph{
    int** adj;
    int vex_num;
    std::vector<Vex> vexList;
    std::map<std::string, int> name_map;

    void initGraph(const int num){
        vexList.resize(num);
        adj = new int*[num];
        for(int i = 0; i < num; ++i){
            adj[i] = new int[num];
        }
        vex_num = num;
    }

    void destroyGraph(){
        if(adj){
            for(int i = 0; i < vex_num; ++i){
                if(adj[i]){
                    delete adj[i];
                    adj[i] = nullptr;
                }
            }
        }
        delete adj;
        adj = nullptr;
    }
};



class Dijkstra{
public:
    Dijkstra(graph &g, const int start);
    ~Dijkstra(){
        delete openList;
        delete closeList;
    }
public:
    // Dijkstra 将使用两种方式实现，一种是A*算法的退化
    void findPath(graph &g, const int start, const int end); 
    void findPath(graph &g, std::string start, std::string end);           
private:

public:
    int* openList;              
    int* closeList;            //
};

Dijkstra::Dijkstra(graph &g, const int start){
    openList = new int[g.vex_num];
    closeList = new int[g.vex_num];
    for(int i = 0; i < g.vex_num; ++i){
        openList[i] = 0;
        closeList[i] = 0;
    }
    closeList[start] = 1;
    g.vexList[start].cost = 0;
}

void Dijkstra::findPath(graph &g, const int start, const int end){
    /// TODO: 1 所有结点当前cost置为很大的数，起点为0
    int now_pos = start;
    while(now_pos != end){
        for(int j = 0; j < g.vex_num; ++j){
            if(closeList[j] != 1 && g.adj[now_pos][j] != INF){      //处理过就不再处理
                int cost = g.vexList[now_pos].cost + g.adj[now_pos][j];
                if(g.vexList[j].cost > cost){                       //更新当前点到其他点的距离
                    g.vexList[j].cost = cost;
                    g.vexList[j].prev = now_pos;
                    openList[j] = 1;
                }  
            }
        }
        int min_cost = INF;
        for(int j = 0; j < g.vex_num; ++j){                     //找openList中cost最小的结点
            if(openList[j]){
                if(g.vexList[j].cost < min_cost){
                    min_cost = g.vexList[j].cost;
                    g.vexList[now_pos].next = j;
                }
            }
        }
        openList[g.vexList[now_pos].next] = 0;
        closeList[g.vexList[now_pos].next] = 1;
        now_pos = g.vexList[now_pos].next;
    }
    now_pos = start;
    while(now_pos != end){
        std::cout << g.vexList[now_pos].name << ">>";
        now_pos = g.vexList[now_pos].next; 
    }
    std::cout << g.vexList[now_pos].name << std::endl;
    delete openList;
    delete closeList;
}



#endif // DIJ_HPP

