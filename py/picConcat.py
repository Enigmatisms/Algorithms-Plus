#!/usr/bin/env python3
#-*-coding:utf-8-*-
# 图片拼接技术
# 实现思想：首先确定首尾（看留白）
# 之后比较两图片（i, j 比较i的最右边一列以及j的最左边一列）
# 得到一个带权重（边权是差异程度）的邻接矩阵，结点为图片
# 求从第一个从起点到终点的最小差异和路径(贪婪算法nec12)

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import json
from copy import deepcopy as dcp

class Concat:
    def __init__(self, path = "./pics/", num = 19):
        self.pics = []                  # 图片集合读pyth取
        for i in range(num):
            length = 2 - i // 10
            pic = cv.imread(path + "0" * length + str(i) + ".bmp")
            pic = cv.cvtColor(pic, cv.COLOR_RGB2GRAY)           # 灰度化
            cv.bitwise_not(pic, pic)                            # 按位取反
            self.pics.append(pic)
        self.cols = len(self.pics)

        self.adjs = np.zeros((self.cols, self.cols))        # 计算邻接矩阵
        for i in range(self.cols):
            for j in range(self.cols):
                self.adjs[i][j] = Concat.compareErr(self.pics[i], self.pics[j])


        self.path = []                                              # 后继集合
        self.prevs = [0 for i in range(self.cols)]                  # 前驱集合
        self.costs = [float("inf") for i in range(self.cols)]

        self.start = 0
        self.end = 0

        self.startNend()            # 确定最左边图片
        self.costs[self.start] = 0
        self.searched = {self.start, self.end}
        self.prevs[self.start] = self.start


    # TODO: 此函数存在需要修改的参数
    def startNend(self):
        # TODO: start_change_cnt end_change_cnt 都是debug使用
        start_change_cnt = 0
        end_change_cnt = 0
        for i in range(self.cols):
            if np.linalg.norm(self.pics[i][:, :2]) == 0:
                self.start = i
                start_change_cnt += 1
            if np.linalg.norm(self.pics[i][:, -2:]) == 0:
                self.end = i
                end_change_cnt += 1
        # 确定图片集合的起始与结尾
        if start_change_cnt > 1 or end_change_cnt > 1:
            print("Change counter: ", start_change_cnt, ", ", end_change_cnt)
        elif start_change_cnt == 0 or end_change_cnt == 0:
            print("Change counter: ", start_change_cnt, ", ", end_change_cnt)
        print("Start and end:", self.start, self.end)

    
    # 比较两张图片
    @staticmethod
    def compareErr(pic1, pic2):
        col1 = pic1[:, -1]
        col1 = col1.astype(float)
        col2 = pic2[:, 0]
        col2 = col2.astype(float)
        error = col1 - col2
        return np.linalg.norm(error, 2)

    def argmin(self):
        mini = float("inf")
        min_pos = 0
        for i in range(self.cols):
            if i in self.searched:
                continue
            if self.costs[i] < mini:
                mini = self.costs[i]
                min_pos = i
        self.searched.add(min_pos)
        return min_pos

    def neighbor(self, costs):
        mini = float("inf")
        min_pos = 0
        for i in range(self.cols):
            if i in self.searched:
                continue
            if costs[i] < mini:
                mini = costs[i]
                min_pos = i
        self.searched.add(min_pos)
        return min_pos
    
    # dij算法
    def dijkstra(self):
        print("Start to dijkstra")
        pos = self.start
        while not pos == self.end:
            for i in range(self.cols):
                cost = self.costs[pos] + self.adjs[pos][i]
                if cost < self.costs[i]:
                    self.costs[i] = cost
                    self.prevs[i] = pos
            pos = self.argmin()
        print("Dijkstra completed.")

    def greedy(self):
        pos = self.start
        self.path.append(pos)
        while len(self.searched) < self.cols:
            pos = self.neighbor(self.adjs[pos, :])
            self.path.append(pos)
        self.path.append(self.end)
        

    # 搜索完成后向左方向的拼接
    def lconcat(self):
        self.result = dcp(self.pics[self.end])
        prev = self.prevs[self.end]
        while not prev == self.start:
            self.result = cv.hconcat((self.result, self.pics[prev]))
            prev = self.prevs[self.end]
        self.result = cv.hconcat((self.result, self.pics[self.start]))
        return cv.bitwise_not(self.result)

    # 搜索完成后向右方向的拼接
    def rconcat(self):
        for i in range(len(self.path) - 1):
            print(str(self.path[i]) + ",", end = "")
        print(str(self.path[i]))
        print("0,", end = "")
        for i in range(len(self.path) - 1):
            print(str(int(self.adjs[self.path[i]][self.path[i+1]] / 10)) + ",", end = "")
        self.result = dcp(self.pics[self.start])
        for node in self.path[1:]:
            self.result = cv.hconcat((self.result, self.pics[node]))
        return cv.bitwise_not(self.result)
        


if __name__ == "__main__":
    plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['font.size'] = 16
    cct = Concat()

    cct.greedy()

    result = cct.rconcat()

    cv.imshow("result", result)

    cv.waitKey(0)

    for i in range(cct.cols):
        for j in range(cct.cols - 1):
            print(str(int(cct.adjs[i][j] / 10))+",", end = "")
        print(str(int(cct.adjs[i][cct.cols - 1] / 10)))

    ys_left = cct.pics[8][:, -1]
    ys_right = cct.pics[14][:, 0]
    xs = np.array([i for i in range(len(ys_left))])
    plt.plot(xs, ys_left, linestyle = "--", label = "左图最右列", color = "black")
    plt.plot(xs, ys_right, linestyle = "-", label = "右图最左列", color = "black")
    plt.xlabel("行数")
    plt.ylabel("像素灰度")
    plt.legend()
    plt.show()

