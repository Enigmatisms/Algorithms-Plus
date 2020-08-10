#!/usr/bin/env python3
#-*-coding:utf-8-*-
# 多切片拼接以及双面拼接的投票算法
# 思路:
## 首先找行的头，由于在行的延伸方向上具有更多的信息，先组合成一行一行的
### 行头：前三列范数和为0(行头集合)
### 行尾集合：后三列范数为0
### 此后进行相似性搜索, 按照行数约束

import numpy as np
import cv2 as cv
from random import choices
from collections import deque
from copy import deepcopy as dcp

class RowCat:
    def __init__(self, rows = 11, cols = 19):
        self.pics = []

        self.rows = rows
        self.cols = cols
        self.size = rows * cols
        self.loadFrom()

        self.path = []
        self.row_heads = deque([], maxlen = rows)
        self.heads = set()
        self.searched = {it for it in self.heads}

        self.getHeads()

        # 最后的票数计算
        # 哈希表, head:[[], [], [], [], ...]
        # value为一个list，表示了行头后的cols - 1个图片的可能取值
        # 内部的list用于投票(每个位置选哪一张图片)
        self.ballots = {head:[[] for i in range(self.cols)] for head in self.row_heads}

        # 此邻接矩阵的计算需要通过已经确定的行头和行尾
        # 其他点到行头的权重为无限，行尾到其他点的权重为无限
        self.adjs = []
        self.k = 180
        
        self.calcAdjs()
        self.degrade = np.array([i / self.cols for i in range(self.cols)])

    # 确定行首
    def getHeads(self):
        for i in range(self.size):
            if cv.countNonZero(self.pics[i][:, 2]) == 0:
                self.row_heads.append(i)
                self.heads.add(i)
        if not len(self.heads) == self.rows:
            print("Unexpected mismatched head number:", len(self.heads), ", ", self.rows)
    
    def loadFrom(self, path = "./pics/"):
        for i in range(self.size):
            length = len(str(i))
            file = path + "0" * (3 - length) + str(i) + ".bmp"
            pic = cv.imread(file)
            pic = cv.cvtColor(pic, cv.COLOR_BGR2GRAY)
            pic = cv.bitwise_not(pic)
            self.pics.append(pic)
        print("Pics loaded from '" + path + "' ." + str(len(self.pics)) + " pics.")

    # 按照概率采样(越近概率值越大)
    def probability(self, field):
        if len(field) == 3:
            res = choices(field, [17/30, 1/3, 3/30])[0]
            return int(res)
        elif len(field) == 2:
            res = choices(field, [0.75, 0.25])[0]
            return int(res)
        elif len(field) == 1:
            return field[0]
        else:
            print("Field is empty. Possible reason: self.k too small with too many conflicts.")
            return -1

    @staticmethod
    def diffCost(p1:np.array, p2:np.array, col = True):
        r, c = p1.shape
        _p1 = dcp(p1)
        _p2 = dcp(p2)
        _p1 = _p1.astype(float)
        _p2 = _p2.astype(float)
        if col:     # 以图片的列作为讨论依据
            pic1 = np.zeros((1, r))
            pic2 = np.zeros((1, r))
            for i in range(c):
                pic1 += i**2 / (c**3) * (_p1[:, i])
                pic2 += (1 - i**2 / c**2) / c * (_p2[:, i])
        else:       # 以行为讨论依据
            pic1 = p1[-1, :]
            pic2 = p2[0, :]
        return np.linalg.norm((pic1 - pic2))

    # 考虑到图像碎片过多，每个图像只保存与其匹配前k
    # 获取前self.k个与index位置对应图片误差最小的图片下标
    def smallestK(self, index):
        tmp = []
        for i in range(self.size):
            if i in self.heads or i == index:
                continue
            tmp.append((RowCat.diffCost(self.pics[index], self.pics[i]), i))
        res = sorted(tmp)[:self.k]
        # 取出元组的[1]分量（对应的是图片的下标）
        return [x[1] for x in res]

    # 计算部分的邻接矩阵
    def calcAdjs(self):
        for i in range(self.size):
            self.adjs.append(self.smallestK(i))

    # 最多取三个误差最小的图片，给probability做随机选择
    def getThree(self, index):
        res = []
        for i in range(self.k):
            if self.adjs[index][i] in self.searched:        # 被查找过 或是为尾部的 不会参与讨论
                continue
            else:
                res.append(self.adjs[index][i])
                if len(res) > 2:
                    return res
        if len(res) == 0:
            print("No valid res for index:", index, " found.")
        return res

    # 投票环节
    def vote(self, loop = 50):
        loops = self.rows * loop    # 循环 loop * rows 次（本题中为11rows）
        for i in range(loops):
            for head in self.row_heads:
                self.singleRow(head)
            self.searched = {it for it in self.heads}       # 重置，开始时就不可搜索self.heads里的图片
            self.row_heads.rotate(1)
        print("Vote process completed.")
        result = np.zeros((self.rows, self.cols))
        result = result.astype(int)
        for i in range(self.rows):      # 众数求第i,j位置票数最多的图片
            for j in range(1, self.cols):
                head = self.row_heads[i]
                tmp = np.bincount(self.ballots[head][j])
                result[i][j] = int(np.argmax(tmp))
        print("Vote count process completed.")
        result[:, 0] = self.row_heads
        for i in range(self.rows):
            output = self.pics[result[i][0]]
            for j in range(1, self.cols):
                output = cv.hconcat((output, self.pics[result[i][j]]))
            cv.imwrite("./output/" + str(result[i][0]) + ".bmp", output)
            
    # 单行处理
    def singleRow(self, head):
        cnt = 1
        now = head
        for i in range(1, self.cols):
            field = self.getThree(now)
            self.searched.add(now)
            now = self.probability(field)
            if not now == -1:
                self.ballots[head][i].append(now)      # 第head行第i列增加now(相当于now多一票)
            cnt += 1

if __name__ == "__main__":
    rct = RowCat()
    rct.vote()


    
    
