#!/usr/bin/env python3
#-*-coding:utf-8-*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as img
from sklearn.neighbors import KNeighborsClassifier      # KNN分类器
from sklearn.externals import joblib                    # 模型保存以及加载
from sklearn.decomposition import PCA


class PCAKNN:
    train_path = "/home/sentinel/trainPics"
    test_path = "/home/sentinel/testPics"
    def __init__(self, dim = 15):
        self.classifier = KNeighborsClassifier(9, 'uniform')
        self.pca = PCA(n_components = dim)

    # 去中心化
    def deCentralized(self, data:np.array):
        rows = data.shape[1]
        # means = np.array([np.mean(data[i, :]) for i in range(rows)])
        for i in range(rows):
            data[:, i] -= np.mean(data[:, i])
            

    # 降维处理，将img降维到dim
    def pcaProcess(self, img:np.array, dim = 15):
        img = img.astype(float)
        # 去中心化
        self.deCentralized(img)
        # 求协方差
        cov = np.transpose(img).dot(img)
        # svd协方差
        u, s, vt = np.linalg.svd(cov)
        # 取前dim 维的特征向量(比如(72, 68)-> (68, 68)的cov， 对应特征矩阵为(68, dim)的shape)
        eig_mat = vt[:, :dim]
        # X.dot(特征向量)
        sample = img.dot(eig_mat)
        # reshape为行向量
        # 将一个dim * dim 的矩阵reshape成一个行向量
        return sample.reshape(sample.size)

        # 降维处理，将img降维到dim
    def standardPCA(self, img:np.array):
        img = img.astype(float)
        sample = self.pca.fit_transform(img)
        return sample.reshape(sample.size)

    # 通过数据以及初始的分类进行训练
    def train(self, data:np.array, label:np.array):
        self.classifier.fit(data, label)
        print("Training completed.")

    # 预测（可以验证
    def predict(self, src:np.array):
        return self.classifier.predict(src)

    def load(self, path = "./model.pkl"):
        self.classifier = joblib.load(path)
        print("Model loaded from: ", path)

    def save(self, path = "./model.pkl"):
        joblib.dump(self.classifier, path)
        print("Model saved to: ", path)

if __name__ == "__main__":
    dim = 20
    model = PCAKNN(dim)

    train_path = "/home/sentinel"

    train = 0
    
    if train:
        cnt = []
        data_set = []

        for i in range(1, 10):
            for j in range(1, 68):
                data = img.imread(train_path + "/trainPics/" + str(i) + "/num" + str(j) + ".png")
                # data_set.append(model.standardPCA(data))
                data_set.append(model.pcaProcess(data, dim))
                cnt.append(i)

        labels = np.array(cnt)
        train_data = np.array(data_set)

        model.train(train_data, labels)

        model.save()
    else:
        model.load()

        test_labels = []
        test_data = []

        for i in range(1, 10):
            for j in range(1, 9):
                data = img.imread(train_path + "/testPics/" + str(i) + "/num" + str(j) + ".png")
                # test_data.append(model.standardPCA(data))
                test_data.append(model.pcaProcess(data, dim))
                test_labels.append(i)

        pred = model.predict(np.array(test_data))
        print(test_labels)
        print(pred)