#-*-coding:utf8-*-

import random
import matplotlib.pyplot as plt

# 各种滤波算法的实现

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
# 


if __name__ == "__main__":
    xs = [i for i in range(200)]
    samp = [random.uniform(0, 1) for i in range(200)]
    out = medianValueFilter(samp, 3)

    plt.plot(xs, samp, color = "red", label = "origin")
    plt.plot(xs, out, color = "blue", label = "filtered")
    plt.legend()
    plt.show()
