#!/usr/bin/env python3
#-coding:utf-8-*-

import matplotlib.pyplot as plt

# 递归实现的归并排序
def merge(lst:list, left, right):
    mid = int((left + right) / 2)
    if left == right:
        return [lst[left]]
    if mid == left:
        if lst[left] > lst[right]:
            return [lst[right], lst[left]]
        else:
            return [lst[left], lst[right]]
    else:
        l_list = merge(lst, left, mid)
        r_list = merge(lst, mid + 1, right)
    l_cnt = 0
    r_cnt = 0
    new_list = []
    while l_cnt < len(l_list) and r_cnt < len(r_list):
        if l_list[l_cnt] <= r_list[r_cnt]:
            new_list.append(l_list[l_cnt])
            l_cnt += 1
        else:
            new_list.append(r_list[r_cnt])
            r_cnt += 1
    if l_cnt < len(l_list):
        new_list.extend(l_list[l_cnt:])
    if r_cnt < len(r_list):
        new_list.extend(r_list[r_cnt:])
    return new_list

# 逆序数对统计
def reverseCounter(lst1:list, lst2:list):
    l_cnt = r_cnt = 0
    rev_cnt = 0
    back = False
    new_lst = []
    while l_cnt < len(lst1) and r_cnt < len(lst2):
        if lst1[l_cnt] <= lst2[r_cnt]:
            new_lst.append(lst1[l_cnt])
            l_cnt += 1
            rev_cnt += r_cnt
        else:
            new_lst.append(lst2[r_cnt])
            r_cnt += 1
            back = True
    if l_cnt < len(lst1):
        new_lst.extend(lst1[l_cnt:])
        rev_cnt += (len(lst1) - l_cnt) * r_cnt
    print(new_lst)
    return rev_cnt

## TODO: 非递归的归并排序以及快速排序的实现
## 完全乱序的逆序数对统计

a = [3, 4, 5, 1, 2, 6, 7, 9, 0, 8, 10, 11, 13, 12]
b = merge(a, 0, len(a) - 1)
print(b)

left = [1, 2, 3, 9, 14, 15, 16]
right = [4, 5, 6, 7, 8, 10, 11, 12, 13, 20]
print(reverseCounter(left, right))