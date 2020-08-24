#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# 其他动态规划问题2 动态规划问题的变体

"""
1. 背包恰好装满问题：
    要求背包一定要被装满，装满的同时可以使装入背包的物品价值最大

2. 分割等和子集问题：
    给定一个数组，将两个数组划分为和相等的两部分 （输出是否能有等和划分解 bool值）
    首先，判定数组所有值的和是否为偶数，如果不是那么显然不能
    再次，如果为偶数，可能可以划分，可知每个划分的和为总和的一半，也即可以转化为一个背包问题
    物品的价值显然已经没有讨论的必要了，需要讨论的是物品的重量（每个数组内的元素的值）
    恰好装满问题
实现的方式完全由：Enigmatisms给出，所以可能不是最佳的
"""

import numpy as np

# 0 - 1 背包的恰好装满问题
# 同样地，可以先出一个递归版本的算法 主要用于状态转移思路上的理清
# 问题的情况是：1. 有没有恰好填充的情况 2. 如果存在多种这样的情况，价值最大的情况是？
# 返回值为 value:int, plausible:bool
def justFit(weights, values, capc, N):
    plausible = False
    if capc == 0:
        return (0, True)
    max_val = 0
    for i in range(N, len(weights)):
        if weights[i] <= capc:
            val, _pl = justFit(weights, values, capc - weights[i], i + 1)
            val += values[i]
            if _pl and val > max_val:
                max_val = val
                plausible = True
    return (max_val, plausible)
# 确实，改成这样就会是对的，毕竟0-1背包是不允许重用的，递归算法中，递归地函数传递本次执行到的位置
# 可以看出，上述递归算法是正确的，而这个实现与dp.py中的递归算法又不一样，dp.py中的实现并未显式地使用for循环


# 非递归实现
# 尝试直接进行空间优化后的justFit算法，而实际上，dp.py中的算法，即使没有恰好放满
# 也是会尝试更新的，虽然输出的都是total[-1]对应的full capacity 位置，但是并不是我们要的结果
def optJustFit(weights, values, capc, N):
    total = np.zeros((capc + 1, 1))
    cps = np.zeros((capc + 1, 1))
    for i in range(N):
        for k in range(capc, weights[i] - 1, -1):
            if i == 0:
                total[k] = values[i]
                cps[k] += weights[i]
            else:
                val = total[k - weights[i]] + values[i]
                if val > total[k]:
                    total[k] = val
                    cps[k] = cps[k - weights[i]] + weights[i]
    temp = total[cps == capc]
    if len(temp):
        return int(max(temp)), True
    return 0, False
# 这样确实就可行了！我们使用的方法是，记录每次加入物品时对应的重量 在cps内
## 为什么这样可行呢？每一次进行更新时（加入第i件物品可以使对应value变大），而我们记录的cps[x]
## cps[x]表示的就是实际使用k-weights[i]容量大小的包，装价值最大的物件时其中被占用的大小
## cps[k - weights[i]]为未加入ith item时的容量，则很好理解了

wts = [3, 4, 6, 3]
vals = [4, 5, 6, 5]
capc = 12
N = 4

val, pl = justFit(wts, vals, capc, 0)

print("Result: value %d, plausible ? %d"%(val, pl))
print("Optimal Result: value %d, plausible ? %d"%(optJustFit(wts, vals, capc, N)))