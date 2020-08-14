#!/usr/bin/env python3
#-coding:utf-8-*-
# SVM 三维点分类

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d as mp3
from sklearn import svm

def make_meshgrid(x, y, h = 0.5):
    # 输入的 x 为一个array , y同样
    # 自适应地选择x, y的最大范围进行网格点的绘制
    # 返回的是二维矩阵, 举例如下
    """
    x = [               y = [
        [1, 2, 3],          [1, 1, 1],
        [1, 2, 3],          [2, 2, 2],   
        [1, 2, 3],          [3, 3, 3],
    ]                   ]
    可用于plot_surface前两个参数传入
    """
    x_min, x_max = x.min() - 1, x.max() + 1
    y_min, y_max = y.min() - 1, y.max() + 1
    xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
                         np.arange(y_min, y_max, h))
    return xx, yy

# 根据平面方程 ax + by + cz + d = 0 取得 已知 x, y下的 z
def getZ(xs:np.array, ys:np.array, a, b, c, d):
    z = []
    for y in ys:
        z.append([ -a / c * x - b / c * y - d / c for x in xs])
    z = np.array(z)
    return z

if __name__ == "__main__":
    mean_pos1 = np.array([6, 6, 6])
    mean_pos2 = np.array([-6, -6, -6])
    mean_pos3 = np.array([0, 0, 0])

    dot_num = 40
    dots1 = np.random.normal(0, 1.44, (dot_num, 3))
    dots2 = np.random.normal(0, 1.44, (dot_num, 3))
    dots3 = np.random.normal(0, 1.44, (dot_num, 3))

    labels = [1 for i in range(dot_num)]
    labels.extend([2 for i in range(dot_num)])
    labels.extend([3 for i in range(dot_num)])
    labels = np.array(labels)

    # 不同的簇
    cluster1 = dots1 + mean_pos1
    cluster2 = dots2 + mean_pos2
    cluster3 = dots3 + mean_pos3

    data = np.zeros((3 * dot_num, 3))
    data[:dot_num, :] = cluster1
    data[dot_num:2 * dot_num, :] = cluster2
    data[2 * dot_num:3 * dot_num, :] = cluster3

    classifier = svm.SVC(kernel = "linear", max_iter = 1000)

    classifier.fit(data, labels)

    print("SVM training process completed.")
    print("Coeffs:")
    print(classifier.coef_)
    print("Support vectors:")
    print(classifier.support_vectors_)
    print("Intercept:")
    print(classifier.intercept_)

    coefs = classifier.coef_
    inc = classifier.intercept_
    sv = classifier.support_vectors_

    fig = plt.figure()
    ax = mp3.Axes3D(fig)

    ax.scatter3D(cluster1[:, 0], cluster1[:, 1], cluster1[:, 2], c = "red", s = 5, label = "cluster r")
    ax.scatter3D(cluster2[:, 0], cluster1[:, 1], cluster2[:, 2], c = "blue", s = 5, label = "cluster b")
    ax.scatter3D(cluster3[:, 0], cluster1[:, 1], cluster3[:, 2], c = "green", s = 5, label = "cluster g")

    xs, ys = make_meshgrid(np.array([-12, 12]), np.array([-4, 12]))

    ax.scatter3D(sv[:, 0], sv[:, 1], sv[:, 2], c = "orange", s = 15, label = "support vectors")
    ax.plot_surface(xs, ys, getZ(xs[0, :], ys[:, 0], coefs[0][0], coefs[0][1], coefs[0][2], inc[0]) , color = "cyan", alpha = 0.5)
    ax.plot_surface(xs, ys, getZ(xs[0, :], ys[:, 0], coefs[1][0], coefs[1][1], coefs[1][2], inc[1]) , color = "purple", alpha = 0.5)
    ax.plot_surface(xs, ys, getZ(xs[0, :], ys[:, 0], coefs[2][0], coefs[2][1], coefs[2][2], inc[2]) , color = "red", alpha = 0.5)

    plt.legend()
    plt.show()

