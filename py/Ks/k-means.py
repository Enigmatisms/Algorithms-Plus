#-*-coding:utf-8-*-
"""
    简单的K-MEANS算法，试试seaborn绘图
    PRML课程复习所作
"""

import numpy as np
import seaborn as sns
from matplotlib import pyplot as plt
from pandas import DataFrame
from random import choices

def generateData(k, pnum = 50):
    centers = np.random.normal(0, 12, (k, 2))
    data = dict()
    index = []
    tmp = []
    for i, ct in enumerate(centers):
        for _ in range(pnum):
            tmp.append(ct + np.random.normal(0, 2, (1, 2)))
            index.append('Class %d'%(i + 1))
    tmp = np.array(tmp, dtype = np.float64).reshape(-1, 2)
    data['x'] = tmp[:, 0]
    data['y'] = tmp[:, 1]
    data['class'] = index
    res = DataFrame(data = data)
    return res

def visualize(data):
    ax = sns.scatterplot(x = 'x', y = 'y', hue = 'class', data = data)

def calcMinPt(pt, cts):
    tmp = np.array([np.linalg.norm(pt - ct) for ct in cts])
    return np.argmin(tmp)


def kmeans(data, k, max_iter = 100):
    pts = np.vstack((data['x'], data['y'])).T
    centers = choices(pts, k = k)
    for i in range(max_iter):
        tmp = [np.zeros((1, 2)) for _ in range(k)]
        cnt = [0.0 for i in range(k)]
        indices = []
        for pt in pts:
            index = calcMinPt(pt, centers)
            tmp[index] += pt
            cnt[index] += 1.0
            indices.append('Class %d'%(index + 1))
        for p in range(k):
            centers[p] = tmp[p] / cnt[p]
        if i == max_iter - 1:
            res = dict()
            res['x'] = data['x']
            res['y'] = data['y']
            res['class'] = indices
            return res
    

if __name__ == "__main__":
    data = generateData(4)
    sns.set()
    plt.subplot(2, 1, 1)
    visualize(data)
    plt.subplot(2, 1, 2)
    result = kmeans(data, 4)
    visualize(result)
    plt.show()
