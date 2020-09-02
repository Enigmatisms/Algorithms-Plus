#!/usr/bin/env python3
#-*-coding:utf-8-*-
# LDA 有类别降维
# 手写实现

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as mp3
from copy import deepcopy as dcp

# 三维三类降维为二维平面点

colors = ("red", "green", "blue")
_labels = ("class 1", "class 2", "class 3")

class LDA:
    def __init__(self, data_dim = 3, tar_dim = 2, label_num = 3):
        self.d_dim = data_dim   # 原维度
        self.t_dim = tar_dim    # 目标维度
        if tar_dim > label_num - 1:
            self.t_dim = label_num - 1
        # 类别个数计数器
        self.label_num = label_num
        self.labels = {i:0 for i in range(label_num)}       # 对应标签的个数
        self.prefix = {i:0 for i in range(label_num)}       # prefix_sum 用于指示起始位置
        self.prefix[label_num] = None                       # 便于循环计算
        self.total = 0

    def reset(self):
        self.prefix = {i:0 for i in range(self.label_num)}
        self.prefix[self.label_num] = None                       # 便于循环计算
        self.labels = {i:0 for i in range(self.label_num)}
        self.total = 0

    # 其中可能存在问题的地方：X是列为特征，行为数据组
    def fit_transform(self, X, y):
        self.reset()
        for i in range(len(y)):
            self.labels[y[i]] += 1
        for i in range(1, self.label_num):      # 起始位置计算
            self.prefix[i] = self.prefix[i - 1] + self.labels[i - 1]
        means = np.zeros((self.label_num, self.d_dim))
        # 计算均值
        data = dcp(X)
        for i in range(self.label_num):
            for j in range(self.d_dim):
                means[i][j] = np.mean(X[self.prefix[i]:self.prefix[i + 1], j])
            # 已经计算完类别 i 的均值，那么数据减去均值
            data[self.prefix[i]:self.prefix[i + 1], :] -= means[i]
        # k(类别数) 个协方差矩阵(大小为原来数据的维数)
        covs = []

        # 计算类内协方差矩阵 并以此计算 Sw
        for i in range(self.label_num):
            part = data[self.prefix[i]:self.prefix[i + 1], :]
            covs.append(np.transpose(data).dot(data))
        Sw = sum(covs)

        if np.linalg.det(Sw) == 0:
            raise ValueError("Data covariance sum is zero")

        # 计算Sb
        center = sum(means) / self.label_num    # 得到数据中心
        Sb = np.zeros((self.d_dim, self.d_dim))
        for i in range(self.label_num):
            diff = means[i] - center
            Sb += self.labels[i] * np.transpose(diff).dot(diff)
        u, s, vt = np.linalg.svd(Sw)
        invSw = np.transpose(vt).dot(np.linalg.inv(np.diag(s))).dot(np.transpose(u))
        res = invSw.dot(Sb)

        # u 与 v均是列为特征向量
        _u, _s, _vt = np.linalg.svd(res)
        return X.dot(_u[:, :self.t_dim])
    
    def draw(self, result):
        plt.figure(2)
        for i in range(self.label_num):
            plt.scatter(result[self.prefix[i]:self.prefix[i + 1], 0],
                        result[self.prefix[i]:self.prefix[i + 1], 1],
                        c = colors[i], label = _labels[i]
            )
        plt.title("LDA result")
        plt.legend()

if __name__ == "__main__":
    amount = [20, 20, 20]

    X1 = np.random.normal(6,  2, (amount[0], 3))
    X2 = np.random.normal(0,  2, (amount[1], 3))
    X3 = np.random.normal(-6, 2, (amount[2], 3))
    lab1 = np.array([0 for i in range(amount[0])])
    lab2 = np.array([1 for i in range(amount[1])])
    lab3 = np.array([2 for i in range(amount[2])])

    fig = plt.figure()
    ax = mp3.Axes3D(fig)

    ax.scatter3D(X1[:, 0], X1[:, 1], X1[:, 2], c = colors[0], label = _labels[0])
    ax.scatter3D(X2[:, 0], X2[:, 1], X2[:, 2], c = colors[1], label = _labels[1])
    ax.scatter3D(X3[:, 0], X3[:, 1], X3[:, 2], c = colors[2], label = _labels[2])
    data = np.vstack((X1, X2, X3))
    label = np.concatenate((lab1, lab2, lab3)).astype(int)

    model = LDA()
    result = model.fit_transform(data, label)

    model.draw(result)

    plt.show()

            
        
        