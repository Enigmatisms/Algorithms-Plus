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
    # 图像PCA不需要均值化，就是个简单的SVD压缩
    def pcaProcess(self, img:np.array, dim = 15):
        img = img.astype(float)
        u, s, vt = np.linalg.svd(img)
        # 取前dim 维的特征向量(比如(72, 68)-> (68, 68)的cov， 对应特征矩阵为(68, dim)的shape)

        # 此处的sample操作是图像绘制时使用的，没有达到实际的压缩保存效果
        # sample = u[:, :dim].dot(np.diag(s[:dim])).dot(vt[:dim, :])

        # 此操作才是真正进行压缩(PCA)
        # sample = u[:, :dim].dot(np.diag(s[:dim]))
        sample = img.dot(u[:, :dim])

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
    dim = 5
    model = PCAKNN(dim)

    train_path = "/home/sentinel"

    train = 1
    
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