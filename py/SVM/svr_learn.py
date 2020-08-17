#!/usr/bin/env python3
#-coding:utf-8-*-
# SVR 支持向量机回归 一种新的回归拟合方式

import numpy as np
import matplotlib.pyplot as plt
from sklearn.svm import SVR
from sklearn.svm import NuSVR
from sklearn.model_selection import GridSearchCV

def poly3func(x, a = 0.1, b = -1.0, c = 2, d = 1):
    return a * x ** 3 + b * x **2 + c * x + d

if __name__ == "__main__":
    size = 80
    xs = np.linspace(0, 10, size)
    ys_true = poly3func(xs)
    ys_data = poly3func(xs) + np.random.normal(0, 3, size) 

    coefs = np.polyfit(xs, ys_data, 3)

    ys_poly = poly3func(xs, *coefs)

    C = 0.1
    gamma = 1.0

    colors = (
        "green", "purple", "orange", "black"
    )

    labels = (
        "SVR poly", "SVR RBF", "SVR sigmoid", "SVR linear"
    )

    models = (
        SVR(C = C, gamma = gamma, kernel = 'poly', degree = 3),
        SVR(C = C, gamma = gamma, kernel = 'rbf'),
        SVR(C = C, gamma = gamma, kernel = 'sigmoid'),
        SVR(C = C, gamma = gamma, kernel = 'linear')
    )
    plt.scatter(xs, ys_data, c = "blue", label = "origin scatter")
    plt.plot(xs, ys_true, color = "blue", label = "ground truth")
    plt.plot(xs, ys_poly, color = "red", label = "polyfit result")
    
    for svr, c, l in zip(models, colors, labels):
        svr.fit(xs.reshape(-1, 1), ys_data.reshape(-1, 1))
        ys_svr = svr.predict(xs.reshape(-1, 1))
        plt.plot(xs, ys_svr, color = c, label = l)

    plt.legend()
    plt.show()