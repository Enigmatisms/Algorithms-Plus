#!/usr/bin/env python3
#-*-coding:utf-8-*-

import sys
import json
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    with open("/home/sentinel/Algorithms-Plus/Ceres/pydata.txt", "r") as rd:
        points = json.load(rd)

    xs = np.array([ pt[0] for pt in points ])
    ys = np.array([ pt[1] for pt in points ])

    plt.scatter(xs, ys, c = "black", marker = "o")


    coeffs = np.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
    fn = np.poly1d(coeffs)

    ys = [fn(x) for x in xs]

    plt.plot(xs, ys, color = "blue")

    plt.show()