#!/usr/bin/env python3
#-*-coding:utf-8-*-
#Metropolis-Hasting 算法

## 总的来说就是：MH算法的建议分布需要能良好地表达被采样分布的性质
## 比如说一个存在正负值的分布（如柯西分布），就必须要使用存在正负值的建议分布
## 分布值比较分散的情况，就不能使用方差过小的建议分布，否则可能出现锁死的现象（见MH类注释）

import numpy as np
from scipy.stats import norm
from functools import partial
import matplotlib.pyplot as plt

PI = 3.141592653589793

class MH:
    """
        MH 算法步骤：
        1. 已知目标的描述函数（由于种种原因，这个概率不方便进行归一化）
        2. 初始化
        3. 选择建议分布，通过建议分布以及接受率关系进行马尔可夫链细致平稳化的迭代
        4. 马尔可夫链收敛后，将会收敛到目标分布上
        5. 建议分布的选择：MH算法并不要求对称，但为什么此处指数分布不能用作建议分布？
        6. 首先，指数函数作为建议分布，那么生成的随机数samp必然比0大，并且根据指数分布的性质
            指数分布不讨论 x <= 0的情况，所以指数分布是不够普适的
        7. 建议分布以及目标分布可以使用pdf（归一化或未归一化均可）进行讨论
        8. 发现的问题：指数函数参数选择与最后的结果存在直接的关联性
            个人认为：指数函数lambda过大，比如选到1.5结果难以跑出来
            问题在于，random.exponential 函数，生成不了过大的样本
            比如下面定义的xexp函数，其中有一半以上的样本在x > 1处
            而lambda越大，曲线越陡峭，集中于很小的值处（<1）
            并且若此时出现了一个较大值self.state 那么会导致什么情况？
            self.pdf 的lambda很大，self.pdf(self.state)很小，而func(self.state)虽然也很小，但不足以抵消self.pdf的效果
            导致alpha = min(1, ...) 一直被限制在很小的位置上，无法更新，self.state会被锁定在一个大值上
        9. 也就是说，MH算法要能很好地收敛，则必然不能出现锁死的情况
            这就要求，建议分布的范围需要稍大一些，不能过于集中，否则无法很好地生成样本
        10. 建议分布的选择对采样结果的影响很大，比如在高斯建议分布上，选择不同的sigma(代表分散程度)
            由于最后我们接受的样本就是依靠选取的参数随机得到的
            sigma小的时候，最后的结果会按照对应形状进行分布，但是其方差也会变小，sigma大的时候同理
            也就是说，我们的采样结果和建议分布生成的随机数关系十分密切，通常，建议分布也是需要进行良好设计的
        11. ***第77行***，注释掉了建议分布，认为q(i|j)/q(j|i) = 1 也可以进行MH采样，为什么？
            大概是这样的：由于对称，我们的独立取样实际上有：从i到j状态 与 从 j到i状态是两个独立的事件
            由于是对称简单随机抽样，i到j，j到i的概率必然是一样的，那么就为1
    """
    def __init__(self, init_iter = 1000, sg = "guass"):
        self.state = 0.5
        self.initialized = False
        self.init_iter = init_iter

        # 使用的建议分布参数
        self.mu = 0
        self.sigma = 2
        self.lamb = 0.4

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
                    # 不进行hasting 优化的称作MCMC采样（朴素的）
                    alpha = min(1, func(samp) / func(self.state))
                        # * self.pdf(self.state) / self.pdf(samp))      
                except ZeroDivisionError:
                    alpha = 1.0
                ## 说明：q(xt + 1 | xt) = q(xt + 1, xt) / q(xt), vice versa
                ## 但在此处，高斯分布不符合独立性条件, q(xt) / q(xt + 1) 不为1
                ## 则为 min(1, p(xt+1) / p(xt) * q(xt) / q(xt+1))
                uni = np.random.uniform(0, 1)
                if uni < alpha:
                    self.state = samp
                    cnt += 1
            print("Initialized")
            self.initialized = True
        else:                           # 已经进入稳态
            samp = self.suggest()
            try:
                alpha = min(1, func(samp) / func(self.state))
                    # * self.pdf(self.state) / self.pdf(samp))
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
    sample_times = 10000
    xs = [[] for i in range(4)]
    for i in range(sample_times):
        # print("Sample iter: %d / %d"%(i, sample_times))
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
        plt.hist(xs[i], bins = 100)
        plt.title(titles[i])
    plt.show()
            
