#!/usr/bin/env python3
#-*-coding:utf-8-*-
#Metropolis-Hasting 算法

import numpy as np
from scipy.stats import norm
from functools import partial
import matplotlib.pyplot as plt

PI = 3.141592653589793

class MH:
    def __init__(self, init_iter = 20, sg = "guass"):
        self.state = 0.5
        self.initialized = False
        self.init_iter = init_iter

        # 使用的建议分布参数
        self.mu = 0
        self.sigma = 1.0
        self.lamb = 1.0

        if sg == "guass":
            self.suggest = partial(np.random.normal, self.mu, self.sigma)
            self.pdf = self.normPdf
        elif sg == "exp":
            self.suggest = partial(np.random.exponential, self.lamb)
            self.pdf = self.pdfexp
    
    # 求高斯分布概率密度
    def normPdf(self, x):
        return 1 / np.sqrt(2 * PI) / self.sigma * np.exp((x - self.mu)**2 / 2 / self.sigma ** 2)

    # 求指数分布概率分布
    def pdfexp(self, x):
        if x <= 0:
            return 0.0
        return self.lamb * np.exp( - self.lamb * x)

    def sampler(self, func):
        if not self.initialized:        # 没有进入稳态
            cnt = 0
            while cnt < self.init_iter:
                samp = self.suggest()
                try:
                    alpha = min(1, func(samp) / func(self.state) * 
                        self.pdf(self.state) / self.pdf(samp))      
                except ZeroDivisionError:
                    alpha = 1.0
                ## 说明：q(xt + 1 | xt) = q(xt + 1, xt) / q(xt), vice versa
                ## 但在此处，高斯分布不符合独立性条件, q(xt) / q(xt + 1) 不为1
                ## 则为 min(1, p(xt+1) / p(xt) * q(xt) / q(xt+1))
                uni = np.random.uniform(0, 1)
                if uni < alpha:
                    self.state = samp
                    cnt += 1
            self.initialized = True
        else:                           # 已经进入稳态
            samp = self.suggest()
            try:
                alpha = min(1, func(samp) / func(self.state) * 
                    self.pdf(self.state) / self.pdf(samp))      
            except ZeroDivisionError:
                alpha = 1.0
            uni = np.random.uniform(0, 1)
            if uni < alpha:
                self.state = samp
        return self.state

# 模拟柯西分布
def cauchy(x):
    return 1 / (1 + x**2)

# 模拟指数分布
def exponent(x):
    if x > 0:
        return np.exp(-x)
    return 0.0

def xexp(x):
    if x <= 0:
        return 0.0
    return x**2 * np.exp( - 0.5 * x )

if __name__ == "__main__":
    mhs = [ MH(),
            MH(),
            MH(sg = "exp"),
            MH(sg = "exp")
    ]
    sample_times = 5000
    xs = [[] for i in range(4)]
    for i in range(sample_times):
        for j in range(4):
            if j & 1:
                xs[j].append(mhs[j].sampler(xexp))
            else:
                xs[j].append(mhs[j].sampler(cauchy))
    
    titles = [
        "Cauchy with suggested distribution: normal",
        "xExp with suggested distribution: normal",
        "Cauchy with suggested distribution: exp",
        "xExp with suggested distribution: exp",
    ]

    for i in range(4):
        plt.figure(i)
        plt.hist(xs[i], bins = 50)
        plt.title(titles[i])
    plt.show()

# 奇怪的结果
## 高斯函数作为建议分布时能较好地对给定分布进行抽样，不管是有没有归一化的函数
## 而指数函数作为建议分布时，只能很好地对指数函数进行抽样
            
