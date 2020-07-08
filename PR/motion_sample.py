#!/usr/bin/env/ python
#-*-coding:utf-8-*-
#Probability Robotics
#Chapter V Programme 5.3
#author: sentinel

#机器人运动模型 位姿生成以及分布采样

import numpy as np
import matplotlib.pyplot as plt

vc = [0.005, 0.001, 0.002]              #机器人速度误差因子
vw = [0.002, 0.001, 0.0005]             #机器人角速度误差因子

## param ut 控制量
# x_last 上一次位姿 

sin = np.sin
cos = np.cos

start = now_pos = np.array([0, 0, 0])   # 起始位姿
ut = [1, 1, 0.15]                       # 标准控制量

iter_times = 3          #控制迭代次数
sample_times = 500      #采样点数

# 分布函数
def sample(var:float):
    if var == 0: return 0 
    if var > 0:
        return np.sqrt(var) * np.random.randn(1)
    else:
        return - np.sqrt(-var) * np.random.randn(1)

# 随机时间间隔
def getRandomDt():
    return abs(np.random.randn(1) / 20)

# 采样算法
def sample_velocity(ut, x_last, dt, rand = True):
    if rand:
        v = ut[0] + sample(vc[0] * ut[0] ** 2 + vw[0] * ut[1] ** 2)
        w = ut[1] + sample(vc[1] * ut[0] ** 2 + vw[1] * ut[1] ** 2)
        r = sample(vc[2] * ut[0] ** 2 + vw[2] * ut[1] ** 2 )
        x = x_last[0] - v/w * sin(x_last[2]) + v/w * sin(x_last[2] + w * dt)
        y = x_last[0] - v/w * cos(x_last[2]) + v/w * cos(x_last[2] + w * dt)
        theta = x_last[2] + w * dt + r * dt
        return np.array([x, y, theta])
    else:
        v = ut[0]
        w = ut[1]
        x = x_last[0] - v/w * sin(x_last[2]) + v/w * sin(x_last[2] + w * dt)
        y = x_last[0] - v/w * cos(x_last[2]) + v/w * cos(x_last[2] + w * dt)
        theta = x_last[2] + w * dt
        return np.array([x, y, theta])

if __name__ == "__main__":
    poses_x = []
    poses_y = []
    pos_x = []
    pos_y = []
    pos = []

    #第一遍循环 初始化 pos(全部位姿)
    for j in range(sample_times):
        dt = 0.5 + getRandomDt()
        sampled = sample_velocity(ut, start, dt)
        pos.append(sampled)
        pos_x.append(sampled[0])
        pos_y.append(sampled[1])
    poses_x.append(pos_x)
    poses_y.append(pos_y)

    #pos建立，此后pos迭代
    for i in range(1, iter_times):
        pos_x = []
        pos_y = []
        for j in range(sample_times):
            dt = 0.5 + getRandomDt()
            sampled = sample_velocity(ut, pos[j], dt)
            pos_x.append(sampled[0])
            pos_y.append(sampled[1])
            pos[j] = sampled
        poses_x.append(pos_x)
        poses_y.append(pos_y)

    # 红色线条为期望运动
    # 蓝色散点为粒子分布
    lx = [start[0]]
    ly = [start[1]]
    now_pos = start
    for i in range(iter_times):
        now_pos = sample_velocity(ut, now_pos, 0.5, False)
        lx.append(now_pos[0])
        ly.append(now_pos[1])
        plt.scatter(poses_x[i], poses_y[i], marker = 'o', color = "blue", s = 4)
    plt.plot(lx, ly, color = "red")

    plt.show()
        

        
