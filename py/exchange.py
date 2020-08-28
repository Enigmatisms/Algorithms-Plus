#!usr/bin/env python3

from copy import deepcopy as dcp

def exchange(arr:list, i, j):
    length = len(arr)
    result = dcp(arr)
    result[(i + 1) % length], result[j % length] = result[j % length], result[(i + 1) % length]
    result[(i + 2) % length:j % length] = result[(j - 1) % length : (i + 1) % length: -1]
    return result

a = list(range(10))

b = exchange(a, 3, 6)
c = exchange(a, 0, 7)
d = exchange(a, 8, 0)
d = exchange(a, 9, 1)

print(b)
print(c)
print(d)