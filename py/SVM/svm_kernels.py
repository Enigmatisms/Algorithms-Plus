#!/usr/bin/env python3
#-coding:utf-8-*-
# 对于不同的SVM核函数的选择
# SVM 图像分类
# PCA_SVM pca的降维作用可以小一些
# 选择不同的核函数进行降维 查看检验集效果
# ROC-AUC 曲线的了解

import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
from sklearn.svm import SVC
import matplotlib.image as img

# 训练数据加载
def loadTrainPics(model, train_path = "/home/sentinel"):
    cnt = []
    data_set = []

    for i in range(1, 10):
        for j in range(1, 68):
            data = img.imread(train_path + "/trainPics/" + str(i) + "/num" + str(j) + ".png").astype(float)
            data_set.append(model.fit_transform(data).ravel())
            cnt.append(i)

    return np.array(data_set), np.array(cnt)

# 验证数据加载
def loadTestPics(model, train_path = "/home/sentinel"):
    test_labels = []
    test_data = []

    for i in range(1, 10):
        for j in range(1, 9):
            data = img.imread(train_path + "/testPics/" + str(i) + "/num" + str(j) + ".png").astype(float)
            test_data.append(model.fit_transform(data).ravel())
            test_labels.append(i)
    return np.array(test_data), np.array(test_labels)

if __name__ == "__main__":
    comp = 20
    pca = PCA(n_components = comp)

    train_data, train_labels = loadTrainPics(pca)
    test_data, test_labels = loadTestPics(pca)

    C = 100
    gamma = 0.001

    # gamma为0.001时RBF核的分类成功率为100% 超过了线性核
    max_iter = 4000
    models = (
        SVC(C = C, max_iter = max_iter, gamma = gamma, kernel = 'rbf'),
        SVC(C = C, max_iter = max_iter, gamma = gamma, kernel = 'poly'),
        SVC(C = C, max_iter = max_iter, gamma = gamma, kernel = 'sigmoid'),
        SVC(C = C, max_iter = max_iter, gamma = gamma, kernel = 'linear')
    )

    model_names = (
        "(RBF kernel)", "(Polynomial kernel)", "(Sigmoid kernel)", "(Linear kernel)"
    )

    print("SVM classifier params: C: %f, gamma: %f"%(C, gamma))
    for model, name in zip(models, model_names):
        model.fit(train_data, train_labels)
        result = model.predict(test_data)

        right = sum((result != test_labels).astype(int))
        ratio = 1 - float(right / len(result))
        print("Model" + name + " fit result: %d faults in total, ratio: %f"%(right, ratio))
        print(result)
    print("Origin labels:")
    print(test_labels)

    plt.show()
    



    