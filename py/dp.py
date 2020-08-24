#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# 动态规划经典问题：0-1 背包, 完全背包, 多重背包问题以及空间复杂度的优化

"""
@author Sentinel (Enigmatisms)
@ 2020.8.22
综合以下讨论，背包DP问题需要明确的几点是：
    1. 0-1背包的横向互不影响 （反向枚举）
        每一个wt对应的储存位点对应的是一个单一的策略
        本次的策略完全由上一个物品的加入情况决定 (不可覆盖性）
    2. 完全背包 横向相互影响性（正向枚举）
        先判定小规模问题，ith item是否加入
        此结果会在大规模问题中被复用
    3. 多重背包 转化为0-1背包处理
"""

import numpy as np

# I should say that, this recursive algorithm is difficult to think of.
# The recursive process

# weights, values : the attributes of items
# capc: current remaining capacity
# item: the 'item'th item
# return: the maximum value
## what if I want to keep track of the selected item ?
def naiveBackPack(weights, values, capc, item):
    if capc <= 0 or item <= 0:
        return 0
    else:
        # The initial case is that, item is the total number of items. Therefore -1 index is valid
        if weights[item - 1] > capc:
            # if capc too small, change another item by index --
            return naiveBackPack(weights, values, capc, item - 1)
        else:
            # 2 possible steps:
            ## 1. this item is not included, for the outcome is less
            ## 2. this item is included, therefore, backpack has only capc - w left
            ## move on to the next item
            return max(
                naiveBackPack(weights, values, capc, item - 1),
                naiveBackPack(weights, values, capc - weights[item - 1], item - 1) + values[item - 1]
            )

# non recursive
# This is still not so clear to you
## You need to do more about this !
def backPack(weights, values, capc, N):
    total = np.zeros((N, capc + 1))        # assume total is set
    
    for i in range(N):
        for wt in range(min(weights), capc + 1):          # wt is the backpack current maximum weight
            if weights[i] <= wt:
                if i == 0:
                    total[0][wt] = values[i]
                total[i][wt] = max(
                    total[i - 1][wt],
                    total[i - 1][wt - weights[i]] + values[i]
                )
            else:
                total[i][wt] = total[i - 1][wt]
    # print(total)
    return np.max(total[-1, :])

# spatial optimization backpack
# 此处只能使用逆向枚举的方式(背包现重逆向枚举)， 分析：
## 如果使用正向枚举， 允许的大小从 min(weights)到capc 几次实验发现值均变大
## 比如说，我本次先更新了wt = a时的total[a]，当内层循环进行时
## 在wt较大时，可能直接用到total[a] 比如 total[b] = max(..., total[b - weight[i]] + values[i])
## 其中 b - weight[i] = a, 则本次计算时，使用了本次的total 结果 total[a]（先行更新了）
## 这会带来什么问题？每一次使用的total 都应该是上一次的，而与本次的计算无关, 这叫做状态转移
## 我们讨论的是一个分配问题，第 i 个物品是否加入时，不同的分配之间不能相互影响
## total[a] 对应的是一种分配方式，它被更新（判定了第i个物体的加入情况）
## 而在循环内被复用了，这是不应该存在的（相当于本次的b分配方式与本次的a分配方式有关）
## 违反了唯一性
def optBackPack(weights, values, capc, N):
    total = np.zeros((capc + 1, 1))     # 只使用 O(capc + 1)的空间

    for i in range(N):
        for wt in range(capc, weights[i] - 1, -1):
            if i == 0:
                total[wt] = values[i]       # 初始赋值
            else:
                # 我们期望是，每一次更新total[wt] 其值都不会比上一次小
                # 比上一次小这种策略是不被包括的
                # 下式表明，本次要么和上次一样（不包括weight[i]）
                # 要么包括（则需要有weights[i]的空位留出）
                total[wt] = max(            # total[wt]的取值在上一次与本次之间进行选择
                    total[wt],                          
                        total[wt],                          
                    total[wt],                          
                        total[wt],                          
                    total[wt],                          
                        total[wt],                          
                    total[wt],                          
                    total[wt - weights[i]] + values[i]
                )
    print(total)
    return float(total[capc])
## 以上说明了动态规划问题（有空间优化时）的两个特征
## 1. max保证了 本次的value和不会比之前的结果差 (对任意一种分配方式)
## 2. 每一轮更新使用的都是上一轮的值 （策略更新的因果性，横向互不干扰）

# completed backpack(knapsack)
# 完全背包问题， 每种物品数量是无限的
def compBackPack(weights, values, capc, N):
    total = np.zeros((capc + 1, 1))

    # 由于还是要对所有的物品进行遍历，并且每种物品的数量是无限的
    # 横向可能存在影响的情况：即当背包容量小wt = a时，决定加入物品i
    # 而wt = b (b > a) 时，会再次使用wt = a时的结果
    # 这就要求我们需要能够有横向影响的正向枚举
    # 其中的原理还需要细说
    for i in range(N):
        for wt in range(weights[i], capc + 1):
            if i == 0:
                total[wt] = values[0]
            else:
                total[wt] = max(
                    total[wt],
                    total[wt - weights[i]] + values[i]
                )
    return float(total[-1])
# 关于完全背包问题这样正向枚举的理由：
## 注意total这个变量，其索引是当前循环的背包大小
## 正枚举时，我们先解决背包容量小的时候的问题，背包容量小代表着问题规模小
## 没有过多的空间可以让我们进行容量的回退 "total[wt - weights[i]] + values[i]"
## total[wt - weights[i]] + values[i] 中 wt - weights[i] 比当前 wt 更小
## total 此位置在正向枚举中被先行更新了，也就是说，先解决了小规模情况下是否加入 ith item 的问题
## 则在此后的大容量情况下，会使用到之前的结果
## 与反向枚举不同的是，反向枚举横向不影响，在容量大时先行决定，再确定小容量是否加入 ith item
## 反向枚举每一次之和上一次循环有关，而正向枚举中，大规模问题会受到小规模问题影响

#--------------------------------------------------------------------#
# 多重背包：每种物品有n[i]个 (非01， 非无限)
# Ns 为每种物品的数目限制
## 再01，完全背包中，对物品数目的限定是由枚举顺序决定的
## 反向枚举由于横向唯一性，物品也是唯一的，正向则与之相反，但两者都没有显式地限定物品数
## 能否在重复使用时，限定横向重复使用的次数？
# 正向枚举，次数限制的尝试
# 其实多重背包有个很简单的想法，就是转换为0-1背包去做 完全背包由于无限性是难以直接做到的
# 但这样 内存的占用会变大
def bndBackPack(weights, values, Ns, capc):
    total = np.zeros((capc + 1, 1))
    wts = [weights[0] for i in range(Ns[0])]
    for i in range(1, len(Ns)):
        wts.extend([weights[i] for j in range(Ns[i])])
    vals = [values[0] for i in range(Ns[0])]
    for i in range(1, len(Ns)):
        vals.extend([values[i] for j in range(Ns[i])])

    for i in range(len(wts)):
        for wt in range(capc, wts[i] - 1, -1):
            if i == 0:
                total[wt] = vals[0]
            else:
                total[wt] = max(
                    total[wt],
                    total[wt - wts[i]] + vals[i]
                )
                # 如果 cnt >= Ns[i], total[wt]不变
    # print(total)
    return float(max(total))

# 内存方面的改进 只是多一重循环
def optBndBackPack(weights, values, Ns, capc):
    total = np.zeros((capc + 1, 1))
    for i in range(len(wts)):
        for j in range(Ns[i]):
            for wt in range(capc, weights[i] - 1, -1):
                if i == 0:
                    total[wt] = values[0]
                else:
                    total[wt] = max(
                        total[wt],
                        total[wt - weights[i]] + values[i]
                    )
                    # 如果 cnt >= Ns[i], total[wt]不变
    return float(max(total))   

if __name__ == "__main__":
    capc = 12
    N = 4
    wts = np.array([3, 4, 6, 4])
    Ns = np.array([1, 2, 1, 4])
    vals = [4, 5, 6, 5]
    print("Naive:", naiveBackPack(wts, vals, capc, N))
    print("Non-recursive:", backPack(wts, vals, capc, N))
    print("Optimal:", optBackPack(wts, vals, capc, N))
    print("Completed:", compBackPack(wts, vals, capc, N))
    print("Bounded:", bndBackPack(wts, vals, Ns, capc))
    print("Optimal Bounded:", optBndBackPack(wts, vals, Ns, capc))