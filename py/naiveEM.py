#!/usr/bin/env python3
#-coding:utf-8-*-
# 朴素EM算法
# 硬币种类以及对应正反面概率的估计

import numpy as np
import matplotlib.pyplot as plt

truth = [0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1]     # O 为 硬币A 1 为硬币 B
t_aup = 0.4
t_bup = 0.7

def generateSample(flip_times = 8):
    samps = []
    for coin in truth:
        s = []
        ub = t_aup
        if coin:
            ub = t_bup
        for i in range(flip_times):
            rd = np.random.uniform(0, 1)
            if rd < ub:             # ub 为概率上界
                s.append(1)         # 1 为正面
            else:
                s.append(0)
        samps.append(s)
    return np.array(samps)

# 输入一个样本以及对应正面概率 求出给定参数下的"期望"
def calcExpectation(samp, prob):
    prod = 1
    for s in samp:
        if s:
            prod *= prob
        else:
            prod *= (1 - prob)
    return prod

if __name__ == "__main__":
    # 当然，样本数量越大最后的预测结果越好
    samples = generateSample(50)
    max_iter = 50
    aup = 0.36
    bup = 0.6
    old_aup = aup
    old_bup = bup

    estimate = np.ones_like(truth)
    criterion = 1e-4

    for it in range(max_iter):
        total_a = cnt_aup = 0
        total_b = cnt_bup = 0
        for i, sp in enumerate(samples):
            # 此处比较的不是期望
            # 我们对某个样本进行不同的假设，基于已有的参数
            # 我们认为在给定参数的估计下（参数越接近真值时），我们估计成立的概率也就越大
            # 根据极大似然估计的思想，应该选择可以让样本出现概率最大的参数
            # 我们观测到的样本 在我们估计的参数下在比别的参数下时更容易被观测到（实际）
            # 在给定参数估计下，某样本为a的联合分布 > b的联合分布（如下）
            # 则说明此样本倾向于为a，根据上一次迭代的参数我们重新划分，再次估计
            # 这样迭代下去，只要a/b样本具有区分度并且样本数量庞大，那么就会收敛
            e_a = calcExpectation(sp, aup)
            e_b = calcExpectation(sp, bup)
            if e_a >= e_b:
                estimate[i] = 0
                total_a += len(sp)
                cnt_aup += sum(sp)
            else:
                estimate[i] = 1
                total_b += len(sp)
                cnt_bup += sum(sp)
        old_aup = aup
        old_bup = bup
        aup = cnt_aup / total_a
        bup = cnt_bup / total_b
        print("Iter (%d / %d): aup: %f, bup: %f"%(it, max_iter, aup, bup))
        if np.sqrt((aup - old_aup) ** 2 + (bup - old_bup) ** 2) < criterion and it > 5:
            print("Convergence before max_iter (%d / %d)"%(it, max_iter))
            break

    # 重新估计每一次硬币抛掷结果
    for it in range(max_iter):
        for i, sp in enumerate(samples):
            e_a = calcExpectation(sp, aup)
            e_b = calcExpectation(sp, bup)
            if e_a >= e_b:
                estimate[i] = 0
            else:
                estimate[i] = 1

    print("Final result:")
    print("Ground truth is aup = %f and bup = %f"%(t_aup, t_bup))
    print("Coins: ", truth)
    print("Estimated probability: a = %f and b = %f"%(aup, bup))
    print("Estimated coins:", estimate)