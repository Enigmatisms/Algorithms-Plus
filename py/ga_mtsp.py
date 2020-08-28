#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# 使用遗传算法对 多旅行商 MTSP 进行求解

import xlrd
import numpy as np
import random as rd
import matplotlib.pyplot as plt
from collections import deque

inf = 10e5

linestyles = ("--", "-", "-.", ":")
colors = ("black", "red", "blue", "green", "cyan", "orange", "purple", "magenta")

# [ ] 对最后的结果进行交换法优化

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
        交叉模式：THSP 启发式虚拟点交叉
        适应度函数：需要另行构造，由于其中涉及到充电的时间，相当于存在与策略相关的点权
    """
    def __init__(self, city, m = 4, p_mut = 0.01, p_crs = 2 / 3, max_iter = 500, pop = 60):
        self.m = m
        self.pop = pop
        self.city = city
        self.p_mut = p_mut
        self.p_crs = p_crs
        self.max_iter = max_iter
        self.chrome_len = city + m - 1
        self.adjs = []
        self.xs = []
        self.ys = []
        self.adjSetup()     # 此步生成带有虚拟结点的邻接矩阵

        self.population = []
        self.cost_sum = 0
        self.costs = np.array([])
        
        self.mini = 0
        self.maxi = 0
        self.generatePopulation()
        self.iter_costs = []
        self.now_iter = 0
    
    def adjSetup(self):
        wb = xlrd.open_workbook(".\\pos2.xlsx")
        st = wb.sheets()[0]
        raw_x = st.col(1)
        raw_y = st.col(2)
        self.xs = [x.value for x in raw_x[1:]]
        self.ys = [y.value for y in raw_y[1:]]
        self.adjs = np.zeros((self.chrome_len, self.chrome_len))
        for i in range(self.city):
            for j in range(self.city):
                if i == j:
                    self.adjs[i][i] = inf
                else:
                    # 由于纬度和经度的关系，纬度单位1能表示经度单位2的距离
                    self.adjs[i][j] = self.adjs[j][i] = (
                        np.sqrt((self.xs[i] - self.xs[j]) ** 2 + (self.ys[i] - self.ys[j]) ** 2)
                    )
        for i in range(-1, - self.m, -1):
            self.adjs[:self.city, i] = self.adjs[:self.city, 0]
            self.adjs[i, :self.city] = self.adjs[0, :self.city]
        self.adjs[-self.m + 1:, -self.m + 1:] = inf     # 互相不可到达

    # 生成随机的初始种群 self.pop 个
    def generatePopulation(self):
        self.population.clear()
        mean = int((self.city - 1) / self.m)
        self.mini, self.maxi = mean - 2, mean + 2     # range(mini, maxi)
        for k in range(self.pop):
            self.population.append(self.generateIndividual())

    def generateIndividual(self):
        new = [0]
        all_city = {i for i in range(1, self.city)}
        for j in range(1, self.m):
            length = rd.randint(self.mini, self.maxi)
            samp = rd.sample(all_city, length)
            new.extend(samp)
            new.append(-j)      # 分割用的虚拟节点
            sub = set(samp)
            all_city -= sub
        new.extend(all_city)
        return new

    # 代价函数（评价使用的适应性函数）
    def costFunc(self, ind):
        cost = 0
        for i in range(1, self.city):
            cost += self.adjs[ind[i - 1]][ind[i]]
        cost += self.adjs[ind[-1]][0]
        self.cost_sum += cost
        return cost

    # 对种群中每一个个体进行评估排序
    def evaluation(self):
        self.cost_sum = 0
        self.costs = np.array(list(map(self.costFunc, self.population)))
        self.costs /= self.cost_sum
        self.iter_costs.append(self.cost_sum)

    # 精英选择机制（快速收敛）
    def elite(self, num):
        temp = list(zip(self.costs, self.population))
        temp = sorted(temp)
        return [x[1] for x in temp[:num]]

    # 精英选择 + 轮盘赌
    def select(self):
        # 随机产生新个体，剩余的self.pop * self.p_crs 个个体由交叉产生
        temp = int(self.p_crs * self.pop)
        self.population = self.elite(self.pop - temp)
        indices = [i for i in range(self.pop - temp)]       # 种群中已有个体的数目
        chosen = set()
        for i in range(temp):
            parents = tuple(rd.choices(indices, k = 3))
            while parents in chosen:
                parents = tuple(rd.choices(indices, k = 3))
            chosen.add(parents)
            self.population.append(
                self.cross3(
                    self.population[parents[0]],
                    self.population[parents[1]],
                    self.population[parents[2]]
                )
            )
        # 选择与交叉完毕，得到新的种群

    def postProcess(self):
        for i in range(self.pop):
            # 基因突变
            p = rd.uniform(0, 1)
            if p < self.p_mut:
                Genetic.mutation(self.population[i], self.chrome_len)
        for i in range(self.pop):
            # 致死的基因型必须替换
            if self.population[i] == None or Genetic.checkFatalMutation(self.population[i], self.chrome_len) == False:
                # print("Fatal gene detected.")
                self.population[i] = self.generateIndividual()
        for i in range(self.pop):
            # 基因顺序修复
            Genetic.fixGeneOrder(self.population[i])
    
    @staticmethod
    def checkFatalMutation(ind:list, length):
        cnt = 0
        if not len(ind) == length:  # 城市数 + 旅行商数 - 1 应该为染色体长度
            print("False")
            return False
        for gene in ind[1:]:
            if gene < 1:
                if cnt < 3:         # 两个旅行商之间的路径长度为 1 或者 0 则为致死的基因型，会被替换
                    return False
                cnt = 0
            else:
                cnt += 1
        if cnt < 3:
            return False
        return True

    # 旅行商顺序在染色体中是固定的 0, -1, -2, ... -(m-1)
    @staticmethod
    def fixGeneOrder(arr:list):
        cnt = 0
        for i in range(len(arr)):
            if arr[i] < 1:
                arr[i] = cnt
                cnt -= 1

    # 处理：变异
    @staticmethod
    def mutation(arr:list, length):
        i, j = rd.choices(range(length), k = 2)
        while arr[i] <= 0 or arr[j] <= 0:
            i, j = rd.choices(range(length), k = 2)
        for x in range(1, 4):
            if x + i >= length or arr[x + i] <= 0:
                break
        for y in range(1, 4):
            if y + j >= length or arr[y + j] <= 0:
                break
        step = min([x, y, abs(i - j) - 1])
        arr[i:i + step], arr[j: j + step] = arr[j: j + step], arr[i: i + step]     # 简单交换

    # 三父代启发式交叉算子 THSP
    def cross3(self, _p1:list, _p2:list, _p3:list):
        lst = [
            deque(_p1),
            deque(_p2),
            deque(_p3)
        ]
        index = rd.choice([0, 1, 2])
        mini = lst[index][0]
        new = [mini]                            # new 为新路径
        selected = {mini}
        for k in range(1, self.chrome_len):
            for i in range(3):                  # 将三个deque开变为一致的
                if not lst[i][0] == mini:
                    self.moveForward(lst[i], mini)
                lst[i].popleft()                # 开头一致后，可以舍弃
            if len(lst[0]) == 0:
                break
            mini = 0
            _min = inf + 1
            for i in range(3):                  
                # 开头一致后，下一个值中, 到new[-1]也即上一个路径点，找出(距离最小)的对应点
                pos = lst[i][0]
                if self.adjs[new[-1]][pos] < _min and not pos in selected:    # 未选择过
                    _min = self.adjs[new[-1]][pos]
                    mini = pos
            # 最后找到的mini为距离上一次加入点最近的点
            new.append(mini)
            selected.add(mini)
        return new

    def moveForward(self, deq, num):
        try:
            index = deq.index(num)
        except ValueError:
            print("Now iter:", self.now_iter)
            print(deq, num)
            raise ValueError
        deq.rotate(len(deq) - index)

    # 迭代出结果在此处(主函数)
    def iteration(self):
        for i in range(self.max_iter):
            self.now_iter = i
            self.evaluation()
            self.select()
            self.postProcess()
        self.evaluation()
        inds = sorted(list(zip(self.costs, self.population)))
        return inds[0]

    def draw(self, result):
        for i in range(len(result)):    # 替换
            if result[i] < 0: result[i] = 0
        result.append(0)
        color_index = -1
        for i in range(len(result) - 1):
            if result[i] == 0:
                color_index += 1
            plt.plot(
                [self.xs[result[i]], self.xs[result[i + 1]]], 
                [self.ys[result[i]], self.ys[result[i + 1]]],
                color = colors[color_index],
                linestyle = linestyles[color_index]
            )
        
        plt.scatter(self.xs, self.ys)

        plt.figure(2)
        xs = np.arange(self.max_iter + 1)
        plt.plot(xs, self.iter_costs)
        plt.show()

if __name__  == "__main__":
    results = []
    for i in range(6):
        ga = Genetic(30, p_mut = 0.02, p_crs = 2/3, max_iter = 500, pop = 100)
        results.append(ga.iteration())
    results = sorted(results)
    print(results)
    ga.draw(results[0][1])


        
        


    
            

            
