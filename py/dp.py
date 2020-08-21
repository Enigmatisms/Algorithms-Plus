#!/usr/bin/env pyhon3
#-*-coding:utf-8-*-
# ZeroOnePack problem

import numpy as np

# I should say that, this recursive algorithm is difficult to think of.
# The recursive process

# weights, values : the attributes of items
# capc: current remaining capacity
# item: the 'item'th item
# return: the maximum value
## what if I want to keep track of the selected item ?
def naiveBackPack(weights, values, capc, item):
    if capc <= 0 or item <= 0:
        return 0
    else:
        # The initial case is that, item is the total number of items. Therefore -1 index is valid
        if weights[item - 1] > capc:
            # if capc too small, change another item by index --
            return naiveBackPack(weights, values, capc, item - 1)
        else:
            # 2 possible steps:
            ## 1. this item is not included, for the outcome is less
            ## 2. this item is included, therefore, backpack has only capc - w left
            ## move on to the next item
            return max(
                naiveBackPack(weights, values, capc, item - 1),
                naiveBackPack(weights, values, capc - weights[item - 1], item - 1) + values[item - 1]
            )

# non recursive
# This is still not so clear to you
## You need to do more about this !
def backPack(weights, values, capc, N):
    total = np.zeros((N, capc + 1))        # assume K is set
    
    for i in range(N):
        for wt in range(min(weights), capc + 1):          # wt is the backpack current maximum weight
            if weights[i] <= wt:
                if i == 0 or wt == 0:
                    total[0][wt] = values[i]
                total[i][wt] = max(
                    total[i - 1][wt],
                    total[i - 1][wt - weights[i]] + values[i]
                )
            else:
                total[i][wt] = total[i - 1][wt]
    print(total)
    return np.max(total[-1, :])

if __name__ == "__main__":
    capc = 10
    N = 3
    wts = np.array([3, 4, 5])
    vals = [4, 5, 6]
    print("Naive:", naiveBackPack(wts, vals, capc, N))
    print("Non-recursive:", backPack(wts, vals, capc, N))