#-*-coding:utf-8-*-
#算法进阶实现

#快速幂算法

def fastPower(base:int, expo:int):
    res = 1
    while expo:
        if (expo & 1) == 1:
            res *= base
        expo >>= 1
        base *= base
    return res

