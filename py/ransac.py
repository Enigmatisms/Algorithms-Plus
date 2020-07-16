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
    # 动态更新最大迭代数值? 由于采样不是完全随机，所以不能这样做

import random as rd
import numpy as np
import matplotlib.pyplot as plt

# 随机数据中会沿着一条直线有较高的分布，输入给定两点，算出标准模型直线

# 生成数据点
def sample(whole:set, inliers:set = set(), searched = set()):
    if len(inliers):
        # 采样的方式： 有一定概率完全从现阶段的最佳模型inliers采样，也有概率完全随机采样
        possi = rd.randint(0, 7)
        if possi <= 1:
            res = rd.sample(inliers, 2)
        elif possi <= 3:
            res = rd.sample(inliers, 1)
            res.extend( rd.sample(whole - inliers, 1))
        else:
            res = rd.sample(whole, 2)
    else:
        res = rd.sample(whole, 2)
    while tuple(res) in searched:
        if len(inliers):
            possi = rd.randint(0, 7)
            if possi == 0:
                res = rd.sample(inliers, 2)
            elif possi <= 2:
                res = rd.sample(inliers, 1)
                res.extend( rd.sample(whole - inliers, 1))
            else:
                res = rd.sample(whole, 2)
        else:
            res = rd.sample(whole, 2)
    searched.add( tuple(sorted(res)) )          # 不重复采样
    return res
            

# 生成x, y在[mini, maxi]区间内的num个点对
def generateSample(num:int, mini:float, maxi:float, line:np.array):
    step = (maxi - mini) / int(0.4 * num)
    xs = [step * i for i in range(int(0.4 * num))]
    samp = { (x, line[0] * x + line[1] + (maxi - mini) / 10 * rd.random() - (maxi - mini) / 20 ) for x in xs}
    rest = num - len(samp)
    noise = set()
    for i in range(rest):
        x = rd.randint(mini * 100, maxi * 100) / 100
        y = rd.randint(mini * 100, maxi * 100) / 100
        noise.add((x, y))
    return samp, noise          # 生成truth 和 noise 共同形成数据集



# 点到直线的误差（使用的不是距离，而是残差s）
def dotLineDist(src:tuple, line:np.array):
    y_hat = line[0] * src[0] + line[1]
    return abs(y_hat - src[1])


def getLine(samp:list):
    xs = np.array([x[0] for x in samp])
    ys = np.array([y[1] for y in samp])
    return np.polyfit(xs, ys, 1)

def approxLineWithInliers(inliers:set()):
    xs = np.array([dot[0] for dot in inliers])
    ys = np.array([dot[1] for dot in inliers])
    return np.polyfit(xs, ys, 1)


if __name__ == "__main__":
    lb = 0
    ub = 100

    data_num = 100                                      # 数据点个数
    iter_num = data_num * (data_num - 1)   
    thresh_coeff = 8   

    paired = set()

    best_model = np.array([0, 0])
    least_err = np.inf

    now_model = np.array([])
    inliers = set()

    stm = getLine([(5, 9), (90, 85)])

    data_truth, data_noise = generateSample(data_num, lb, ub, stm)
    data = data_truth | data_noise
    epoch = 0
    while epoch < iter_num:
        if least_err < np.inf:
            samp = sample(data, inliers, paired)
        else:
            samp = sample(data, searched = paired)

        now_model = getLine(samp)
        now_inliers = set()
        ref = tuple(samp)[0]
        for dot in data:
            if dotLineDist(dot, now_model) < abs(ub - lb) / 20:
                now_inliers.add(dot)
        
        if len(now_inliers) > int(data_num / thresh_coeff):
            now_model = approxLineWithInliers(now_inliers)
            total_err = 0
            for dot in data:
                total_err += dotLineDist(dot, now_model)
            if total_err < least_err:
                best_model = now_model
                least_err = total_err
                inliers = now_inliers
        epoch += 1
    
    outliers = data - inliers

    outx = [dot[0] for dot in outliers]
    outy = [dot[1] for dot in outliers]
    inlx = [dot[0] for dot in inliers]
    inly = [dot[1] for dot in inliers]
    ttx = [dot[0] for dot in data_truth]
    tty = [dot[1] for dot in data_truth]

    stm_fn = np.poly1d(stm)
    best_fn = np.poly1d(best_model)

    xs = [i for i in range(lb, ub + 1)]
    stm_ys = [stm_fn(i) for i in xs]
    best_ys = [best_fn(i) for i in xs]

    plt.scatter(outx, outy, c = "black", marker = "o", label = "Outliers")
    plt.scatter(inlx, inly, c = "green", marker = "o", label = "Inliers")
    plt.scatter(ttx, tty, c = "yellow", marker = "+", label = "Truth")
    plt.plot(xs, stm_ys, color = "blue", label = "Truth LSF")
    plt.plot(xs, best_ys, color = "red", label = "RANSAC")

    outx.extend(inlx)
    outy.extend(inly)

    lsq = np.polyfit(outx, outy, 1)
    lsq_fn = np.poly1d(lsq)
    lsqy = [lsq_fn(x) for x in range(lb, ub + 1)]
    plt.plot(xs, lsqy, color = "cyan", label = "LSF")
    plt.legend()
    plt.title("RANSAC versus LS")
    plt.show()

        
    
                    


