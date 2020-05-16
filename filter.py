#-*-coding:utf8-*-

import random
import matplotlib.pyplot as plt
from collections import deque

# 各种滤波算法的实现

g_old = 0           #一阶滞后滤波里使用的历史数据保存（python没有静态函数）

#1 限幅滤波
#每一个数据与上一个数据进行比较，假如差值超过阈值，则为上一个值
def ampFilter(src, amp):
    lst = []
    old = 0
    for i in range(len(src)):
        if abs(src[i] - old) > amp:
            lst.append(old)
        else:
            lst.append(src[i])
            old = src[i]
    return lst

#2 中值平均滤波（去掉最大值最小值后取平均）
# 可能用队列或是循环取模的方式采样稍微好些
# 可过滤掉异常输入，去掉最大和最小值后取平均
# radius 滤波取值半径(大于等于2)
def medianMeanFilter(src, radius = 2):
    lst = []
    length = len(src)
    lst.extend(src[0 : radius])
    for i in range(radius, length - radius):
        temp = sorted(src[i - radius : i + radius + 1])
        lst.append( sum(temp[1 : len(temp) -1 ]) / (2 * radius + 1) )
    lst.extend(src[length - radius : length])
    return lst

#3 中值滤波
# 对于变化缓慢的数据中出现异常大或者异常小的数据具有较好的处理能力
#排序后直接取中间的那一个
def medianValueFilter(src, radius = 2):
    lst = []
    length = len(src)
    lst.extend(src[0 : radius])
    for i in range(radius, length - radius):
        temp = sorted(src[i - radius : i + radius + 1])
        lst.append( temp[radius] )
    lst.extend(src[length - radius : length])
    return lst

#4 滑动平均滤波(直接递推)
# 相当于滤波器2的一个优化版
# 不再实现了（因为deque不能实现改变maxlen），使用deque即可
# 我现在也不想做有关数据结构的事情
def slideWindowFilter(src, radius = 2):
    out = []
    length = 2 * radius + 1
    hold = deque([], length)
    out.extend( src[ : radius])
    hold.extend(src[ : length])
    for i in range(radius, len(src) - radius - 1):
        out.append(sum(hold) / length)
        hold.append(src[i + radius + 1])
    out.append(sum(hold) / length)
    out.extend( src[len(src) - radius:] )
    return out

#5 限幅平均滤波
# 数据先限幅，后平均
# 这个没有什么好说的，可以结合slideWindowFilter做这个
def slideAmpWindowFilter(src, thresh = 1.0, radius = 2):
    out = []
    out.append(src[0])
    for i in range(1, len(src)):          #倒置
        if abs(src[i] - src[i - 1]) > thresh:
            out.append(src[i - 1])
        else:
            out.append(src[i])
    length = 2 * radius + 1
    hold = deque([], length)
    hold.extend(out[ : length])
    for i in range(radius, len(out) - radius - 1):
        out[i] =  sum(hold) / length
        hold.append(out[i + radius + 1])
    out[len(out) - radius - 1] = sum(hold) / length
    return out

#6 即时的处理
# 一阶滞后滤波器
# 这个地方还有讨论的空间
# 尝试把这个和离散低通滤波联系起来
# 起始二阶的构造也比较简单
def firstOrderFilter(now, ratio = 0.5):
    return (1 - ratio) * now + g_old * ratio

def firstOrderFilter_Batch(src, ratio = 0.5):
    out = []
    out.append(src[0])
    for i in range(1, len(src)):
        out.append( (1 - ratio) * src[i] + ratio * src[i - 1] )
    return out

#7 加权递推平均
# 即滑动平均算法的加权改进处理
# 对新采样的数据加以更大的权重


if __name__ == "__main__":
    xs = [i for i in range(200)]
    samp = [random.uniform(0, 1) for i in range(200)]
    out = firstOrderFilter_Batch(samp, 0.4)

    plt.plot(xs, samp, color = "red", label = "origin")
    plt.plot(xs, out, color = "blue", label = "filtered")
    plt.legend()
    plt.show()
