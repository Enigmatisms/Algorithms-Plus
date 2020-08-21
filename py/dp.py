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
        if weights[item] > capc:
            # if capc too small, change another item by index --
            return naiveBackPack(weights, values, capc, item - 1)
        else:
            # 2 possible steps:
            ## 1. this item is not included, for the outcome is less
            ## 2. this item is included, therefore, backpack has only capc - w left
            ## move on to the next item
            return max(
                naiveBackPack(weights, values, capc, item - 1),
                naiveBackPack(weights, values, capc - weights[item], item - 1) + values[item]
            )

# non recursive version
# I think, the integer feature is essential
# In the recursive version, we implicitly accumulate the values
# The accumulated value is propagated during return and function stacks
# But within one function, we need storage media
# notice that, there could be sparsity in weights
# e.g weights = [0, 50, 100], capc = 120
# If we use capc to represent the whole space, it would be a waste
# We want to directly use the number of all the possible combinations
# Then we don't have to use a list but a hash table
def backPack(weights, values, capc, N):
    min_val = min(weights)
    total = np.zeros((itemN, capc - min_val))
    # traverse N items
    for i in range(N):
        # traverse possible weights (decreasingly)
        for wt in range(capc, min_val - 1, -1):
            # adding ith item will surpass the limit
            if capc < wt + weights[i]:
                total[i][wt - min_val] = total[i - 1][wt - min_val]
            else:
                total[i][wt + weights[i] - min_val] = max(
                    total[i - 1][wt - min_val] + values[i],
                    total[i - 1][wt - min_val]
                )

if __name__ == "__main__":
    capc = 10
    N = 3
    wts = [3, 4, 5]
    vals = [4, 5, 6]
    print("Naive:", naiveBackPack(wts, vals, capc, N))
    print("Non-recursive:", backPack(wts, vals, capc, N))