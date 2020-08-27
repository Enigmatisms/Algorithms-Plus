#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# 使用遗传算法对 多旅行商 MTSP 进行求解

import xlrd
import numpy as np
import random as rd
import matplotlib.pyplot as plt

inf = 10e7

class Genetic:
    """
        MTSP 问题的遗传算法求解
        city 城市个数
        m 旅行商个数
        p_mut 变异概率
        p_crs 交叉概率
        max_iter 最大代数
        pop 一次迭代内的最大种群数
        选择：轮盘赌式选择
        染色体构造：type = list, 十进制编码，其中设置了m - 1个虚拟位点
        交叉模式：HSP 启发式虚拟点交叉
        适应度函数：需要另行构造，由于其中涉及到充电的时间，相当于存在与策略相关的点权
    """
    def __init__(self, city, m = 4, p_mut = 0.01, p_crs = 0.5, max_iter = 500, pop = 60):
        self.m = m
        self.pop = pop
        self.city = city
        self.p_mut = p_mut
        self.p_crs = p_crs
        self.max_iter = max_iter
        self.adjs = []
        self.adjSetup()     # 此步生成带有虚拟结点的邻接矩阵

        self.population = []
        self.sorted = []
        self.cost_sum = 0
        self.costs = []
    
    # 借助xlrd，读取地图数据，求出结点之间的欧几里得距离
    # 同时注意虚拟点的计算，注意我们将真实起始点重排为0位置
    # 以四旅行商为例 -1, -2, -3为旅行商虚拟结点，虚拟结点间的距离极大（虚拟结点是绝对不能相连的）
    # 虚拟结点与真实结点 0 到其他任意结点（除其他虚拟节点外）的距离均一致，可以直接复制
    def adjSetup(self):
        pass 

    # 生成随机的初始种群 self.pop 个
    def generate(self):
        self.population.clear()
        mean = int((self.city - 1) / self.m)
        mini, maxi = mean - 2, mean + 2     # range(mini, maxi)
        for k in range(self.pop):
            new = [0]
            all_city = {i for i in range(1, self.city)}
            rem = self.city - 1
            for j in range(1, self.m):
                length = rd.randint(mini, maxi)
                samp = rd.sample(all_city, length)
                new.extend(samp)
                new.append(-j)      # 分割用的虚拟节点
                sub = set(samp)
                all_city -= sub
                rem -= length
            new.extend(all_city)
            self.population.append(new)

    # 代价函数（评价使用的适应性函数）
    def costFunc(self, ind):
        cost = 0
        for i in range(1, self.city):
            cost += self.adjs[i - 1][i]
        cost += self.adjs[self.city - 1][0]
        self.cost_sum += cost
        return cost

    # 对种群中每一个个体进行评估排序
    def evaluation(self):
        self.cost_sum = 0
        self.costs = list(map(self.costFunc, self.population))
        temp = sorted(list(zip(self.costs, self.population)))
        self.sorted = [val[1] for val in temp]

    # 精英选择 + 轮盘赌
    def select(self):
        self.population.clear()
        # 首先通过self.pop 个个体按照概率选择最佳的1/3
        # 根据1/3的较优个体 由 crossover 生成 self.pop 个个体(说不定会重复)
        # 进行变异
        pass
        
        


    
            

            
