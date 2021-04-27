import numpy as np

infinity = 1e8

def getAdjs(paths, node_num):
    adjs = [[infinity for i in range(node_num)] for j in range(node_num)]
    for (x, y, cost) in paths:
        adjs[x][y] = cost
        adjs[y][x] = cost
    return adjs

"""
    迪杰斯特拉算法
"""
def Dijkstra(adjs, start):
    length = len(adjs)
    parent = [-1 for i in range(length)]           # 父结点 路径记录
    done = [False for i in range(length)]          # 是否找到最短路
    cost = [infinity for i in range(length)]   # 出发结点到当前结点的成本
    mini_pos = 0
    mini_val = infinity
    done[start] = True
    cost[start] = 0
    for i in range(length):                    # 初始化
        val = adjs[start][i]
        if val >= infinity or start == i: continue
        cost[i] = val
        parent[i] = start
        if val < mini_val:
            mini_val = val
            mini_pos = i
    while not all(done):
        done[mini_pos] = True
        for i in range(length):
            if done[i]: continue
            val = adjs[mini_pos][i]
            if val >= infinity: continue
            update = val + cost[mini_pos]
            if update < cost[i]:
                cost[i] = update
                parent[i] = mini_pos
        if all(done): break
        mini_val = infinity
        for i in range(length):
            if done[i]: continue
            val = cost[i]
            if val < mini_val:
                mini_val = val
                mini_pos = i
    return cost, parent

def getPath(names, parent, start, end):
    path = []
    now = end
    while now != start:
        path.append(now)
        now = parent[now]
    path.append(start)
    path.reverse()
    for x in path[:-1]:
        print(names[x] + "->", end='')
    print(names[end])

if __name__ == "__main__":
    nodes = ['A', 'B', 'C', 'D', 'E', 'F']
    paths = [
        (0, 1, 3), (0, 2, 1), (1, 2, 1), (1, 3, 1), (1, 4, 4), (2, 3, 3), (2, 4, 6), (3, 4, 1), (3, 5, 3), (4, 5, 1)
    ]
    adjs = getAdjs(paths, 6)
    cost, parent = Dijkstra(adjs, 0)
    getPath(nodes, parent, 0, 5)