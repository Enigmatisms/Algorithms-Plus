#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# 其他动态规划问题：有意识地进行 递推 小规模化处理

"""
最大分割问题
    输入一个字符串（数字字符串）长度为n
    在其中插入k（k < n - 1）个乘号，可以使这个式子的积最大
    个人感觉，这像一个0-1背包
    这个问题模型大概是这样的
    首先，对于任意一个拆分的位置，比如说在第k位加入了一个乘号
    那么前0 - k中的乘积必须最大，k+1 到末尾的乘积也必须最大
    【已经划分】【乘号】【未划分】为结构，但是实际上这样说还是有问题的
    如何确定此处的【乘号】就能最好地分开已经划分和未划分区域？
    另一种理解：一个长度为n字符，此问题等价于：前x位进行了最优的划分，划分为k-1段
    最后留下来的(x+1到n)这个整数，为第k段。那么我们已经获得了更小子规模的问题
    k段最大乘积 = 前x位的k-1段最大乘积 * 最后的一段整数
    那么x将是我们需要讨论的东西
"""

import numpy as np

# 首先是递归版本的实现 将 in 分解为seg 段
# list为已经拆分的字符串
def maxProduct(arr:list, seg):
    if len(arr) - seg == 1:
        val = 1
        for x in arr:
            val *= int(x)
        return val
    elif seg == 0:
        return int(''.join(arr))
    max_val = 0
    for k in range(len(arr) - 1, seg, -1):
        val = maxProduct(arr[:k], seg - 1) * int(''.join(arr[k:]))
        if val > max_val:
            max_val = val
    return max_val
# 动态规划问题的递归版本并不难写，主要是要搞清楚如何进行规模缩小
# 也就是状态转移：有一个合理的状态转移就能减小问题规模
# 比如此处的转移方程：前x位分为k-1段与最后一段的积
# 则转化为对x进行遍历，逐步缩减 k -> k-1 ... 的一种方式


# 非递归方式
# 此处如果要储存，可能真的需要像我想的那样，使用二维数组进行储存
def nonMaxProduct(arr:list, seg):
    total = np.ones((len(arr), seg + 1))       # x, k 为储存大小
    for x in range(len(arr)):           
        for k in range(0, min(x + 1, seg + 1)):       # 其中, x表示计算arr[:x]的情况， k <= x 表示最大可插入的乘号个数
            if k == 0:
                total[x][0] = int(''.join(arr[:x + 1]))
                continue
            if x == k:
                for i in range(x + 1):
                    total[x][k] *= int(arr[i])
                continue
            if x > 0 and k > 0:
                for y in range(k - 1, x):       # 划分块的数目不能与乘号数目（k）相等比如：[0, 1]插入两个乘号(最多只能有一个), 并且参与划分的元素至少要大于等于乘号数目+1
                    val = total[y][k - 1] * int(''.join(arr[(y + 1):(x + 1)]))
                    if val > total[x][k]:
                        total[x][k] = val
    return int(total[-1][-1])      
# 以上的思想是：储存之前的计算结果：
# 其中的x表示:正在分析第x位的情况（0， 1， 2... x）位全部考虑
## 存在几个约束：1. k = 0时，也就是[0:x + 1]个字符中不插入乘号,直接合并
## 2. x == k时：前x+1位(0, 1, ..., x)插入x个乘号，相当于字符间两两分割
## 此后由DP的思想：每一次计算都要使用上一次的结果
## 我们在计算前x+1位分割为k段时，可以直接查：x+1(0, 1, ..., x)中，前y+1(0, 1, ..., y)分割为k-1段 最后的 y+1 到 x位合并为一个数字
## 也就是在其子规模问题中进行搜索，找到k-1位的分割 * 最后一段整数 的最大值
## 空间可以优化吧


# 非递归，内存空间优化版本 内存占用为O(n), 时间复杂度为O(n^2*k) 其中n为字符串长度，k为需要插入的乘号数（划分为k+1段）
# 个人认为，如果需要优化为一个一维数组，必须进行反向枚举，否则会导致横向的相互影响
# 很容易被循环变量的选取绕晕，简单说一下流程
## 使用反向枚举法，与上面不同的是，外层循环为k
## 由于每一次分割（比如分割成k段）都与分割成k-1段时有关，举个例子，1234分成2段则与每一部分不进行分割时相关
## 比如说: 1234在(0, 1, 2, 3)不进行分割得到的值为(1, 12, 123, 1234)
## 我们需要找到一个乘号放置的位置，让我们已经计算得到的值 * 其后的一整段整数 最大，这样就可以得到（上一次段数 + 1（最后一段整数））的最大分割
## 那也就是说，我们每次需要对每个位置进行分k段的讨论，并且，所有使用的分段情况都需要是上一次的分段情况
## 由于计算高位的total值时需要使用到低位的total值，则在高位更新前，低位不能更新，故需要使用反向枚举
## 注意特殊情况： 1. 不分割的k == 0: 需要将整段字符当作数字
## 2. x == k: x 个数字分为x段，则每段只能有一个数字，如果k = 0时不进行continue,将会整段的值被覆盖 比如 12 被覆盖成 1 * 2
## 保证横向不覆盖性以及单调递增性
def optMaxProduct(arr:list, seg):
    total = np.ones((len(arr), 1))
    total[0] = int(arr[0])
    for k in range(0, seg + 1):
        for x in range(len(arr) - 1, k - 1, -1):
            if k == 0:
                total[x] = int(''.join(arr[:x + 1]))
                continue
            if x == k:
                total[x] = 1
                for i in range(x + 1):
                    total[x] *= int(arr[i])
            else:
                max_val = 0
                for y in range(x - 1, k - 1, -1):
                    val = total[y] * int(''.join(arr[(y+1):(x+1)]))
                    if val > max_val:
                        max_val = val
                total[x] = max_val
    return int(total[-1])

string = "92342098"
seg = 4

# 检验结果是对的
print(maxProduct(list(string), seg))
print(nonMaxProduct(list(string), seg))
print(optMaxProduct(list(string), seg))



