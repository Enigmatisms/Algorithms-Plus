#!/usr/bin/env python3
#-coding:utf-8-*-
# SVM 三维点分类

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d as mp3
from sklearn import svm

if __name__ == "__main__":
    mean_pos1 = np.array([3, 4, 5])
    mean_pos2 = np.array([-2, -3, -4])
    mean_pos3 = np.array([2, -2, 0])

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

    fig = plt.figure()
    ax = mp3.Axes3D(fig)

    ax.scatter3D(cluster1[:, 0], cluster1[:, 1], cluster1[:, 2], c = "red", s = 5)
    ax.scatter3D(cluster2[:, 0], cluster1[:, 1], cluster2[:, 2], c = "blue", s = 5)
    ax.scatter3D(cluster3[:, 0], cluster1[:, 1], cluster3[:, 2], c = "green", s = 5)

    plt.show()

