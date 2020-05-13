#-*-coding:utf8-*-

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

#2 中值平均滤波
# 可过滤掉异常输入，去掉最大和最小值后取平均
# radius 滤波取值半径(大于等于2)
def medianAverageFilter(src, radius = 2):
    lst = []
    length = len(src)
    lst.extend(src[0 : radius])
    for i in range(radius, length - radius):
        temp = sorted(src[i - radius : i + radius + 1])
        lst.append( sum(temp[1 : len(temp) -1 ]) )
    lst.extend(src[length - radius + 1 : length])
    return lst
