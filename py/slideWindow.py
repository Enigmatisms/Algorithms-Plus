#!/usr/bin/env python3
#-*-coding:utf-8-*-
# 滑动平均灯条打击模式的验证

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anm
from collections import deque

# Kalman Filter implemented via Python
# Use a simple 6 * 6 CA model
class KalmanFilter:
    def __init__(self, max_len):
        self.dq = deque([], maxlen = max_len)
        if max_len & 1:
            self.med_dq = deque([], maxlen = max_len)
        else:
            self.med_dq = deque([], maxlen = max_len + 1)
        self.slide_sum = 0                      # 滑动平均均值保存
        self.statePost = np.zeros((6, 1))
        self.statePre = np.zeros((6, 1))        # 先验状态
        self.preCov = np.eye(6)            # 先验协方差
        self.postCov = np.eye(6)           # 后验协方差
        self.measureNoise = np.eye(6)  # 后验误差 
        self.transNoise = np.eye(6)    # 转移误差
        self.measureMat = np.eye(6)           # 观测转移矩阵（线性系统Kalman不会改变）

    def kalmanSetUp(self, preCov, measureErrorPost = np.zeros((6, 6)), transErrorPost = np.zeros((6, 6))):
        self.preCov = preCov
        if not measureErrorPost == np.zeros((6, 6)):
            self.measureErrorPost = measureErrorPost
        if not transErrorPost == np.zeros((6, 6)):
            self.transErrorPost = transErrorPost

    # CA 模型下的转移矩阵（不是固定时间间隔的控制量输入）
    @staticmethod
    def transition(self, dt):
        return np.array([
            [1, 0, dt, 0, 0.5 * dt ** 2, 0],
            [0, 1, 0, dt, 0, 0.5 * dt ** 2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]    
        ])

    # trans 状态转移矩阵
    # ctrl 控制量加入
    # 预测阶段
    # Kalman 复习： 流程
    def predict(self, trans:np.array, ctrl:np.array):
        # 计算高斯先验均值(高斯噪声的均值为zeros)
        self.statePre = trans.dot(self.statePost) + ctrl

        # 计算先验协方差 P(pre) = A * P(last post) * At + R(状态转移噪声协方差)
        self.preCov = trans.dot(self.postCov).dot(np.transpose(trans)) + self.transNoise

    def correct(self, measure):
        # 测量的总先验协方差：状态的先验协方差为P时，则 C * P(pre) * Ct + Q, Q为测量噪声大小
        tmp = self.measureMat.dot(self.statePre).dot(np.transpose(self.measureMat)) + self.measureNoise

        # SVD分解，直接求逆效率较低，对对角矩阵s求逆比较简单
        u, s, vt = np.linalg.svd(tmp)

        # 求测量先验协方差的逆 A = U *S * Vt, inv(A) = V * inv(S) * Ut
        tmp2 = np.transpose(vt).dot(np.linalg.inv(s)).dot(np.transpose(u))

        # 计算Kalman增益 K(Gain) = P(pre) * Ct * 测量先验协方差的逆
        gain = self.preCov.dot(np.transpose(measure)).dot(tmp2)

        # 后验（信息融合：控制与测量）的高斯均值： x(post) = x(pre) + K * (z - C * x(pre))
        self.statePost = self.statePre + gain.dot(measure - self.measureMat.dot(measure))

        # 转移后验协方差 (I - K * C) * P(pre)
        self.postCov = (np.eye(6) - gain.dot(self.measureMat)).dot(self.preCov)

        # 以此实现了：
        ## 二源信息（测量，控制）的融合：使用控制进行预测推定，使用测量进行修正
        ## 针对高斯误差过滤效果好
        # TODO: 修改实现，实现任意维度状态的KF
        # TODO: 进行曲线验证(使用有高斯噪声以及椒盐噪声混合的单维度曲线进行验证)
        # TODO: 实现一个低通滤波器
        
    # 滑动平均滤波器
    # 其实中值滤波可以应用于静止装甲板滤波
    def slideWindow(self, val):
        if len(self.dq) < self.dq.maxlen:
            self.slide_sum = (self.slide_sum * len(self.dq) + val) / (len(self.dq) + 1)  
            self.dq.append(val)
            return self.slide_sum
        else:
            self.slide_sum = (self.slide_sum * self.dq.maxlen - self.dq.popleft() + val) / self.dq.maxlen
            self.dq.append(val)
            return self.slide_sum

    # 中值滤波
    def medianFilter(self, val):
        self.med_dq.append(val)
        lst = sorted(self.med_dq)
        size = len(self.med_dq)
        if size < self.med_dq.maxlen:
            if size & 1:
                return lst[int((size - 1) / 2)]
            else:
                return (lst[int(size / 2)] + lst[int(size / 2) - 1]) / 2
        else:
            return lst[int((self.med_dq.maxlen - 1) / 2)]


if __name__ == "__main__":
    kf = KalmanFilter(12)

    ## 滤波算法的验证
    sz = 100
    xs = np.array([i/10 for i in range(sz)])
    ys_raw = np.array([np.random.normal(0, 1) for i in range(sz)])
    ys_slide = np.array([kf.slideWindow(x) for x in ys_raw])
    ys_med = np.array([kf.medianFilter(x) for x in ys_raw])

    plt.scatter(xs, ys_raw, c = "red", s = 8, marker = "+")
    plt.scatter(xs, ys_slide, c = "blue", s = 8, marker = "+")
    plt.scatter(xs, ys_med, c = "yellow", s = 8, marker = "+")
    plt.plot(xs, ys_raw, color = "black", label = "Origin")
    plt.plot(xs, ys_slide, color = "green", label = "Slide Window")
    plt.plot(xs, ys_med, color = "purple", label = "Median Filter")

    plt.legend()
    plt.show()

