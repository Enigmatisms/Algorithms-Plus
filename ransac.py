#!/usr/bin/env python3
#-*-coding:utf-8-*-
# author hqy
# date 2020.7.13
# RANSAC linear approx algorithm

# 解决如下问题:
    # 1. 有约束的采样算法(已经得到了一个较好的模型，如何利用这个较好信息来更快收敛到真实值呢)
        # 约束可以用 从内点采样的方式获得
    # 2. 使用set来保存已经采样过的点对,那么n个点最多有n(n-1)/2次搜索
    # 3. 误差阈值如何设置
        # 根据分布大小, 距离 < 最大边界的1/20
    # 4. 如何判定模型的好坏？多少个内点才算一个可以考虑的模型？
        # 先行采用：大于总点数1/10
    # 动态更新最大迭代数值

import random as rd
import numpy as np
import matplotlib.pyplot as plt

# 随机数据中会沿着一条直线有较高的分布，输入给定两点，算出标准模型直线
def standardModel(src1:tuple, src2:tuple):
    return np.array([])

# 生成数据点
def sample(whole:set, inliers:set = set()):
    return []

# 生成x, y在[mini, maxi]区间内的num个点对
def generateSample(num:int, mini:float, maxi:float, line:np.array):
    return set()

# 计算点到直线的距离
# 通过直线以及直线上一点计算
def dotLineDist(src:tuple, dot:tuple, line:list):
    return 0.0

def getLine(sample:list):
    return np.array([])

def approxLineWithInliers(inliers:set()):
    return np.array([])

def adaptiveIterNum(num_inl:int):
    return 1000

if __name__ == "__main__":
    lb = 0
    ub = 100

    data_num = 100                                      # 数据点个数
    iter_num = data_num * (data_num - 1) / 2    
    thresh_coeff = 10   


    best_model = np.array([0, 0])
    least_err = np.inf

    now_model = np.array([])
    inliers = set()

    stm = standardModel((5, 9), (90, 85))

    data = generateSample(data_num, lb, ub, stm)

    epoch = 0
    while epoch < iter_num:
        if least_err < np.inf:
            samp = sample(data, inliers)
        else:
            samp = sample(data)

        now_model = getLine(samp)
        now_inliers = set()
        ref = list(samp)[0]
        for dot in data:
            if dotLineDist(dot, ref, now_model) < abs(ub - lb) / 20:
                now_inliers.add(dot)
        
        if len(now_inliers) > int(data_num / thresh_coeff):
            now_model = approxLineWithInliers(now_inliers)
            total_err = 0
            for dot in now_inliers:
                total_err += dotLineDist(dot, ref, now_model)
            if total_err < least_err:
                best_model = now_model
                least_err = total_err
                inliers = now_inliers
                iter_num = adaptiveIterNum(len(inliers))        # 自适应计算迭代次数
        epoch += 1
    
    outliers = data - inliers

    outx = np.array([dot[0] for dot in outliers])
    outy = np.array([dot[1] for dot in outliers])
    inlx = np.array([dot[0] for dot in inliers])
    inly = np.array([dot[1] for dot in inliers])

    stm_fn = np.poly1d(stm)
    best_fn = np.poly1d(best_model)

    xs = [i for i in range(lb, ub + 1)]
    stm_ys = [stm_fn(i) for i in xs]
    best_ys = [best_fn(i) for i in xs]

    plt.scatter(outx, outy, c = "black", marker = "o")
    plt.scatter(inlx, inly, c = "green", marker = "o")
    plt.plot(xs, stm_ys, color = "blue")
    plt.plot(xs, best_fn, color = "red")

    plt.show()

        
    
                    


