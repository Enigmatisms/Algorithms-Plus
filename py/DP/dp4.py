#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# 其他动态规划问题3 

"""
换一种思路
    不要搞太麻烦的实现
    比如dp3.py中讨论的，恰好装满问题
    可以采用别的思路：比如不合法的背包大小
"""

import numpy as np

inf = 0xffffffff
half_inf = 0x7fffffff

# 思路是：包存在许多不合法的状态：
## 比如，包此时的容量就是9, 存在装入方式[4, 5]则合法，若只存在[4, 4]这样的装入方式
## 说明包无法在容量为9时装满，那么就设为非法
def optJustFit(weights, values, capc, N):
    total = np.array([-inf + 1 for i in range(capc + 1)]).astype(int)
    total[0] = 0
    for i in range(N):
        for k in range(capc, weights[i] - 1, -1):
            total[k] = max(
                total[k - weights[i]] + values[i],
                total[k]
            )
            ## 上式说明，如果val能取正常值，需要 k - weights[i]是合法的
            ## 有必要每个子问题都是装满的吗？注意k在本问题中的意义
            ## total[k] 当 k - weights[i] 不合法，那么加了weights[i]的这个地方必定还是不能装满背包
            ## 所以本题的思路是这样的：查看加入 ith 物品时，total[k]是否合法
            ## 即 total[k - weights[i]]是否合法，如果不合法，那么 + weights[i] 得到的 k 也不是满的
            ## 也就是说，不合法的装入方式是无法产生恰好装满的合法解的
    
    # 为了显示方便，转换一下
    for i in range(N):
        if total[i] < 0:
            total[i] = -1
    print(total)
    return total[capc], (total[capc] > 0)
# 这个问题给了我们什么样的启示？
# 原来的实现，k - weights[i] 或者一直向前递推都可能是未装满的情况推演而来的
# 那么可想而知 capc - weights[i] 也可能是未装满 但value较大的值
## 我们引入了合法性限制，让递推关系上只保留有合法值，这也能保证最后的结果必然是背包装满的
## 个人觉得，这一步确实巧妙 用了很简单的一步，没有修改逻辑就完成了要求
## 当然，我们会有这样的疑问：不充满的背包在后续步骤中无法被挽救吗？假如可以挽救，岂不是少讨论了某些情况？
## 实际上，我们应该这样理解：我们是反向枚举的，由大规模向小规模变化
## 由于这是个恰好装满问题，大规模情况下，我们是默认k已经恰好充满
## 那么capc = k - weights[i] 这个小规模问题，我们认为背包大小为capc时也是充满的
## 而之前某一次计算时，某个 k - weights[i] 的结果是未充满的情况
## 因为我们的total[k] 表示的是 背包容量为k且充满时，最大的价值
## 自然，出现了不充满的结果时，会由于我们“默认充满”而导致错误传递到最后

# 对于上面的想法我们需要进行一些小的改动，以进行计数
# 比如下面的例子，可以看出，如果1，2，4只存在一次，那么会有2*3*6 = 36 种不同情况
# 计数如何进行表示？需要建立values 或 weights 与 counter 的关系
# 实际上由于values为ones向量，那么很可能计数直接与total有关
# 个人觉得这个是可以在求解完成之后根据total反推的 但是正推的方法应该会更妙
# 大规模计数将是小规模计数的和，比如 a 可以通过 b, c, d 得到， 则cnt(a) = sum(cnt(b, c, d))
def countJustFit(weights, S):
    N = len(weights)
    total = np.array([0 for i in range(S + 1)])     # 计数时无需设为 -inf 
    total[0] = 1                                # 为0背包恰好“装满”
    for i in range(N):
        for k in range(S, weights[i] - 1, -1):
            # 每次相当于 自增 1 * total[k - weights[i]]
            # 1 表示 ith 物品 选取时的可能情况 total[k - weights[i]] 表示
            # 小规模问题恰好装满的情况总数
            # 不同装入方式之间是相加的，相同装入方式之间向下传递是相乘的
            # + 体现在 += 环节，将不同 weights[i] 相加，而 * 体现在total[k - weights[i]]
            # 每一个 "单一的" 情况 * t[k - w[i]]
            total[k] += total[k - weights[i]]
    return total[S]
# 关于这个思想的结果，我们可以推一下
# 在i = 0 遍历时，只有能直接放入weights[0] 处存在非0值
# i = 1时，存在 只 i=1 放入与 i=1, i=0 同时放入的情况 w[1]+w[0]处记录
# 此后每加入一件物品，都与之前所有的放入情况有关，由于加入物品属于不同的放入策略
# 即使加入的物品存在weight相同的情况，我们仍然认为策略是不同的
# 不同的策略之间遵循加法法则，同一策略之间为乘法法则
# 我们只需要理解，当加入 ith 物品时的策略数变化情况即可
# 已知 weights[i] = b, 则 capc - b 容量对应total为对应规模的全部可能数量
# 根据乘法 / 加法法则就已经能推出来了

# 那么解决了恰好装满问题的可能性，最大值以及计数问题 dp3.py 中所说的问题则十分容易解决

"""
训练：零钱兑换问题
    给定一个价值 val
    给定一些面值的零钱，每个面值的零钱都是无限的
    为了让兑换者方便使用，我们想让兑换者尽可能换得少的零钱数（比如硬币数尽量少）
    （可以知道，100元若全部兑换成1元则非常麻烦）
    来自 Leetcode 322
"""

# 很显然是一个恰好兑换的问题，而这次希望的是价值最低（包恰好装满，装入的物品尽可能少）
## 1. 非计数的恰好装满问题，需要根据小规模恰好装满问题的结果出发 （也就是设置合法性）
## 2. 最小价值，需要把max改成min
## 3. 完全背包问题，横向相互影响，需要正向枚举
## 感觉这题也可以用回溯法 先从大面值开始，使用栈
## 增加难度：输出结果
## 输出结果需要进行回溯，增加一个前驱表
## 增加难度2：每种面值零钱有限（转换为0-1恰装满背包问题）
def change(weights, val):
    total = np.array([inf for i in range(val + 1)])
    prev = [0 for i in range(val + 1)]
    total[0] = 0
    for i in range(len(weights)):
        for k in range(weights[i], val + 1):
            v = total[k - weights[i]] + 1
            if v < total[k]:
                total[k] = v
                prev[k] = weights[i]            # 前驱位置只需要 k - weights[i]
    # 输出结果
    if total[val] < half_inf:
        res = []
        now = val
        while now >= 0:
            res.append(prev[now])
            now -= prev[now]
            if now == 0:
                break
        return res, True
    return [], False


arr = [8, 2, 4, 5, 3, 4, 2, 2, 1, 1, 4, 4, 4, 4]
vals = np.ones((len(arr), 1))

print("OPT: ", optJustFit(arr, vals, 30, len(arr)))

arr = [1, 1, 2, 2, 3]       # 正解是五种
print("Counter:", countJustFit(arr, 6))

arr = [1, 2, 5, 10, 20, 50]
money = 192
print("Result for %d:"%(money), change(arr, money))


