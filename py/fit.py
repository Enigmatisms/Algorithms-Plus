#!/usr/bin/env python3
#-*-coding:utf-8-*-

import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

norm = 692.97944
coeff = np.array([-6.81255495e-13, 2.23967582e-09, -2.61811669e-06, 1.26270659e-03,
 -2.02581731e-01])


def func(x, a, b):
    return x / (a * x ** 2 + b)

def diff_func(x, a, b):
    return (-a * x ** 2 + b)/((a * x ** 2 + b) ** 2) / 1382.085566405469

def part_func(x):
    return sum(np.array([x**4, (x)**3, (x)**2, x, 1] * coeff))


if __name__ == "__main__":
    print("coeff:", coeff)
    xs_raw = sorted(np.array([3.2, 3.4, 3.8, 2.3, 3.5, 1.7]))
    xs = np.array( [1239.8 / x for x in xs_raw ])
    ys = np.array(sorted(np.array([423.6, 570, 839.6, 241.2, 621.44, 165.3])))

    a, b = optimize.curve_fit(func, xs, ys)[0]

    xs_ = np.array([x for x in range(300, 1040)])
    ys_ = np.array([func(x, a, b)  / 1382.085566405469 for x in xs_])

    dys = np.array([ diff_func(x, a, b) for x in xs_])
    pys = np.array([ part_func(x) for x in xs_])

    tys = dys + pys
    
    sub1 = plt.subplot(211)
    plt.scatter(xs, ys / 1382.085566405469, c = "black", marker = "o",  label = "Data points")
    plt.plot(xs_, ys_, color = "red", label = "Fit function")

    sub2 = plt.subplot(212)
    plt.plot(xs_, pys, color = "red", label = "Light Absorption")
    plt.plot(xs_, tys, color = "blue", label = "Total Differential")
    plt.plot([300, 1040], [0, 0], color = "black", label = "zero")

    print("x / (%f * x ^ 2 %f)"%(a, b))
        
    sub1.legend()
    sub2.legend()

    # sub1.set_title(r"氧化还原反应和截止波长(与禁带宽度有关)的关系")
    # sub2.set_title(r"评价函数的导数")

    plt.show()
