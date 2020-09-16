#!/usr/bin/env python3
#-*-coding:utf-8-*-
# 绘制PID控制曲线

import json
import sys
import numpy as np
import matplotlib.pyplot as plt

def readAndDraw(path = "./output.txt"):
    with open(path, "r") as re:
        dots = np.array(json.load(re))
    
    ts = dots[:, 0]
    exps = dots[:, 1]
    acts = dots[:, 2]

    plt.plot(ts, exps, c = "blue", label = "control expectation")
    plt.plot(ts, acts, c = "red", label = "actual output")
    plt.legend()
    plt.grid(axis = "both")
    plt.show()

if __name__ == "__main__":
    readAndDraw(sys.argv[1])