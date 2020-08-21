#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# ZeroOnePack problem

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
    total = np.zeros((N, capc + 1))        # assume K is set
    
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
            if wt >= weights[i]:
                if i == 0:
                    total[wt] = values[i]       # 初始赋值
                else:
                    # 我们期望是，每一次更新total[wt] 其值都不会比上一次小
                    # 比上一次小这种策略是不被包括的
                    # 下式表明，本次要么和上次一样（不包括weight[i]）
                    # 要么包括（则需要有weights[i]的空位留出）
                    total[wt] = max(            # total[wt]的取值在上一次与本次之间进行选择
                        total[wt],                          
                        total[wt - weights[i]] + values[i]
                    )
    # print(total)
    return float(total[capc])

if __name__ == "__main__":
    capc = 12
    N = 4
    wts = np.array([3, 4, 5, 3])
    vals = [4, 5, 6, 5]
    print("Naive:", naiveBackPack(wts, vals, capc, N))
    print("Non-recursive:", backPack(wts, vals, capc, N))
    print("Optimal:", optBackPack(wts, vals, capc, N))