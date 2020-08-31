#!/usr/bin/env python3
#-*-coding:utf-8-*-
#模拟退火算法

import numpy as np
import matplotlib.pyplot as plt

class Anneal:
    """
        模拟退火\n
        - func 代价函数(只计算状态本身，做差不在func中实现)\n
        - update 更新解的策略\n
        - state 初始状态 根据问题而给定\n
        - max_iter 最大迭代次数\n
        - max_idle 当超过max_idle次拒绝接收新解后，算法自动结束\n
        - search_range 产生新解的比例因数\n
        - init_temp 初始温度\n
        ---
        为了能够接入TSP算法，这样设计\n
        此类设计成：默认用于解决一元函数在给定区间上的最值问题\n
        func 为代价函数，可以传入TSP问题类的代价函数\n
        update 在解决一元函数最值问题时，传入一个tuple，为解搜索的开区间\n
        解决别的问题时（比如TSP问题），传入一个函数，对TSP得到的解进行更新\n
        ---
        初始温度是需要与问题匹配的吧，比如一个问题的代价函数变化值最大才0.001\n
        选择初始温度为100显然不合适\n
    """
    def __init__(self, func, update, state, max_iter = 10000, max_idle = 100, search_range = 0.8, init_temp = 50):
        self.max_iter = max_iter
        self.max_idle = max_idle
        self.idle_cnt = 0
        self.search_range = search_range
        self.init_temp = init_temp
        self.cost_func = func
        self.state = state
        self.state_cost = func(state)

        self.cost_track = []
        self.state_track = []
        self.naive = True                                       # 指示，此类是否解决的是朴素的最值问题
        if type(update) == tuple or type(update) == list:       # 只对2维问题做出适配
            self.lb, self.ub = update
            def update():
                res = np.random.uniform(- search_range, search_range)
                new_state = self.state + res
                while new_state >= self.ub or new_state <= self.lb:
                    res = np.random.uniform(- search_range, search_range)
                    new_state = self.state + res
                return new_state
        else:
            self.naive = False
        self.update = update

    def anneal(self):
        for i in range(self.max_iter):
            temp = self.init_temp / (1 + i)
            new_state = self.update()
            cost = self.cost_func(new_state)
            diff = cost - self.state_cost
            if diff < 0:        # 新解的代价小 直接接受
                self.state = new_state
                self.state_cost = cost
                self.idle_cnt = 0
            else:
                if self.metropolis(diff, temp):
                    self.state = new_state
                    self.state_cost = cost
                    self.idle_cnt = 0
                else:
                    self.idle_cnt += 1
                    if self.idle_cnt > self.max_idle:
                        print("Converge before max_iter. Maximum idle counter reached.")
                        break
            self.state_track.append(self.state)
            self.cost_track.append(self.state_cost)
        return self.state, self.state_cost

    def metropolis(self, diff, temp):
        p =  np.exp(- diff / temp)
        uni = np.random.uniform(0, 1)
        return uni < p

    def getStateCost(self):
        # 用作绘图 state_track即是横坐标，cost_track为纵坐标
        return self.state_track, self.cost_track

def func_to_solve1(x):
    return 0.6 * x**2 - 2 * np.cos(4 * x)

def func_to_solve2(x):
    return 6 * np.sin(4*x) / (x**2 + 1)

if __name__ == "__main__":
    xs = np.linspace(-3, 3, 50)
    ys1 = func_to_solve1(xs)
    ys2 = func_to_solve2(xs)

    
    bound = (-3, 3)
    an1 = Anneal(func_to_solve1, bound, -2.9)
    an2 = Anneal(func_to_solve2, bound, -2.9)
    an3 = Anneal(func_to_solve2, bound, 2.9)

    print("Annealing result for func1:", an1.anneal())
    print("Annealing result for func2 with init -2:", an2.anneal())
    print("Annealing result for func2 with init 2.4:", an3.anneal())

    plt.plot(xs, ys1, label = "0.6x^2 - 3cos(4x)", color = "red")
    plt.scatter(*an1.getStateCost(), label = "annealing process", c = "blue")
    plt.legend()

    plt.figure(2)
    plt.plot(xs, ys2, label = "6sin(4x) / (x^2 + 1)", color = "red")
    plt.scatter(*an2.getStateCost(), label = "init at -2.9", c = "blue")
    plt.scatter(*an3.getStateCost(), label = "init at 2.9", c = "green")
    plt.legend()

    plt.show()