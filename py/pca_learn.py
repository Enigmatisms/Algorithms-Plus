#!/usr/bin/env python3
#-coding:utf-8-*-
#简单的三维pca尝试

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d as mp3
import random as rd
import json

if __name__ == "__main__":
    # 使用 x + y + z = 3 作为点云生成平面
    # x, y, z均为随机的(正态分布)
    num = 60
    xs = 4 * np.random.randn(num) + 1
    ys = 4 * np.random.randn(num) + 1
    zs = np.array([3 - xs[i] - ys[i] + rd.gauss(0, 2) for i in range(num)])      

    fig = plt.figure(1)
    ax = mp3.Axes3D(fig)

    # 绘制三维散点图
    ax.scatter3D(xs, ys, zs, c = "red", s = 5)

    cx = np.mean(xs)
    cy = np.mean(ys)
    cz = np.mean(zs)

    xs_c = xs - cx
    ys_c = ys - cy
    zs_c = zs - cz

    cov = np.cov( np.array([xs_c, ys_c, zs_c]))
    u, s, v = np.linalg.svd(cov)

    trans = v[:2, :]

    data = np.array([xs_c, ys_c, zs_c])

    proj_data = trans.dot(data)

    xs_p = proj_data[0, :]
    ys_p = proj_data[1, :]

    fig2 = plt.figure(2)
    plt.scatter(xs_p, ys_p, c = "red")

    plt.show()

