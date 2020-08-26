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
    return (int(max_val), plausible)
# 确实，改成这样就会是对的，毕竟0-1背包是不允许重用的，递归算法中，递归地函数传递本次执行到的位置
# 可以看出，上述递归算法是正确的，而这个实现与dp.py中的递归算法又不一样，dp.py中的实现并未显式地使用for循环


# 非递归实现
# 尝试直接进行空间优化后的justFit算法，而实际上，dp.py中的算法，即使没有恰好放满
# 也是会尝试更新的，虽然输出的都是total[-1]对应的full capacity 位置，但是并不是我们要的结果
# 注意，此求解结果是错误的
# 比如我们下面给出的反例 capc = 30 递归算法给出的是正确的答案
# 错误出现在 cps 的求取
# 在小规模下，这个算法是正确的，规模越大，越容易出现 elif 行注释所说的情况
# 此方法没有记录的能力，只有cps[capc]才可能等于capc
def optJustFit(weights, values, capc, N):
    total = np.zeros((capc + 1, 1)).astype(int)
    cps = np.zeros((capc + 1, 1)).astype(int)
    for i in range(N):
        for k in range(capc, weights[i] - 1, -1):
            if i == 0:
                total[k] = values[i]
                cps[k] = weights[i]
            else:
                val = total[k - weights[i]] + values[i]
                if val > total[k]:
                    total[k] = val
                    cps[k] = cps[k - weights[i]] + weights[i]
                elif val == total[k] and cps[k - weights[i]] + weights[i] == capc:
                    # 此处存在一个小bug，我们默认当value相等的时候是不需要取出的
                    # 但可能相同的value对应了不同的重量，这时我们希望尽可能装满
                    # 但实际上，假如与更远的情况相关，这种处理方法(可能)也是不行的
                    cps[k] = capc
    print("Cps with capc %d:"%(capc), cps.ravel())
    temp = cps[cps == capc]
    print(temp)
    if len(temp):
        return int(max(temp)), True, len(cps[cps.ravel() == capc])
    return 0, False, 0
# 这样确实就可行了！我们使用的方法是，记录每次加入物品时对应的重量 在cps内
## 为什么这样可行呢？每一次进行更新时（加入第i件物品可以使对应value变大），而我们记录的cps[x]
## cps[x]表示的就是实际使用k-weights[i]容量大小的包，装价值最大的物件时其中被占用的大小
## cps[k - weights[i]]为未加入ith item时的容量，则很好理解了
## 以上实现还存在着较大的问题 -- 写的太复杂 / 大规模问题是错的 / 不能计数

# 应用 等和分割算法 （Leetcode 416题）
# 有一个数组，尝试将此数组分成两个和相等的数组，输出是否可能（bool）
def identicalSum(arr):
    s = sum(arr)
    if s & 1 == 1:
        return False
    capc = int(s / 2)
    N = len(arr)
    vals = np.ones((capc + 1, 1)).astype(int)
    _, pl, _ = optJustFit(arr, vals, capc, N)
    return pl

# 目标和问题 TargetSum Leetcode 494
## 有一个非负数组array，一个给定值S(正数)，需要我们在数组的每个值前后添加正负号，使得加了符号的每一个值的之和等于s
## 求 有几种这样的情况。分析可知，暴力搜索显然可以，但为O(2^n)级别的
## 想到一个递归的方法，但是复杂度为 2^n 别递归了
## 这个题有个很妙的解法: 划分集合为A， B， 可知
## A 集合为正号，B集合为负号集（值是正的，减去负的）, 则A + B = sum(arr), A - B = S ! 则解方程可知
## A = (sum(arr) + S) / 2 这就要求：1. sum(arr) > S, 2. sum(arr) + S 为偶数 3.（sum(arr) - S）/ 2 不小于数组的最小值
## 转化问题为：恰好装满的0 - 1 背包 并求数量 
def targetSum(arr, S):
    s = sum(arr)
    if (s + S) & 1 == 1:
        return 0
    if s - S < min(arr):
        return 0
    if s - S < 0:       # B 集合内不能出现负数 可以为0
        return 0
    capc = int((s + S) / 2)
    print("Target capc: ", capc)
    N = len(arr)
    vals = np.ones((N, 1))
    _, _, num = optJustFit(arr, vals, capc, N)
    return num
    
wts = [3, 4, 6, 3]
vals = [4, 5, 6, 5]
capc = 12
N = 4

val, pl = justFit(wts, vals, capc, 0)

print("Result: value %d, plausible ? %d"%(val, pl))
print("Optimal Result: value %d, plausible ? %d with %d method(s)"%(optJustFit(wts, vals, capc, N)))

arr1 = [3, 4, 5, 2, 3, 3, 6, 1, 1, 3, 3, 4, 2]
arr2 = [3, 4, 5, 2, 3, 3, 6, 1, 1, 2, 4, 3, 3]
arr3 = [2, 4, 3, 3, 3, 4, 5, 2, 3, 3, 6, 1, 1]
arr4 = [4, 5, 7, 4]

print("Arr1: ", identicalSum(arr1))     # 存在等和分割
print("Arr2: ", identicalSum(arr2))     # 存在等和分割 
print("Arr3: ", identicalSum(arr3))     # 存在等和分割 
print("Arr4: ", identicalSum(arr4))     # 不存在等和分割

arr = [8, 2, 4, 5, 3, 4, 2, 2, 1, 1, 4, 4, 4, 4]

S = 12
print("Target sum: %d, with %d different methods." %(S, targetSum(arr, S)))