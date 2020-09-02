#!/usr/bin/env python3
#-*-coding:utf-8-*-
# LDA 调库以及 matplotlib 常用方法学习
# LDA 也是可以用作分类的（至少sklearn 存在predict选项）
# 个人感觉，分类这个事情比较简单？降维以后进行最邻近（对均值求最邻近）

import numpy as np
import matplotlib.pyplot as plt
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA

# 绘制分类结果（平面）
def drawClass(model, X1, X2, X3):
    xmax = max([max(X1[:, 0]), max(X2[:, 0]), max(X3[:, 0])])
    xmin = min([min(X1[:, 0]), min(X2[:, 0]), min(X3[:, 0])])
    ymax = max([max(X1[:, 1]), max(X2[:, 1]), max(X3[:, 1])])
    ymin = min([min(X1[:, 1]), min(X2[:, 1]), min(X3[:, 1])])
    xs = np.linspace(xmin - 0.5, xmax + 0.5, 50)
    ys = np.linspace(ymin - 0.5, ymax + 0.5, 50)
    xx, yy = np.meshgrid(xs, ys)

    # 平铺于(-5, 5) (x, y) 的点
    temp = np.transpose(
        np.vstack((xx.ravel(), yy.ravel()))
    )

    prob = model.predict(temp)
    prob = prob.reshape(xx.shape)

    plt.pcolormesh(xx, yy, prob, alpha = 0.5)
    plt.scatter(X1[:, 0], X1[:, 1], marker = 'o', c = "red", label = "class 1")
    plt.scatter(X2[:, 0], X2[:, 1], marker = 'o', c = "green", label = "class 2")
    plt.scatter(X3[:, 0], X3[:, 1], marker = 'o', c = "blue", label = "class 3")

    # TODO:进行检测 (是否存在假阳性)


# 生成三个随机二维类
def generateSamples():
    X1 = np.random.normal(0, 2, (np.random.randint(28, 32), 2))
    y1 = np.array([1 for i in range(len(X1))])
    X1 += np.array([4, 3])
    X2 = np.random.normal(0, 2, (np.random.randint(28, 32), 2))
    y2 = np.array([2 for i in range(len(X2))])
    X2 += np.array([-4, 3])
    X3 = np.random.normal(0, 2, (np.random.randint(28, 32), 2))
    y3 = np.array([3 for i in range(len(X3))])
    X3 += np.array([0, -4])
    return X1, X2, X3, np.concatenate((y1, y2, y3))


if __name__ == "__main__":
    lda = LDA()
    X1, X2, X3, labels = generateSamples()
    lda.fit(np.vstack((X1, X2, X3)), labels)

    drawClass(lda, X1, X2, X3)

    plt.show()