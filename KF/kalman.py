#!/usr/bin/env python3
#-*-coding:utf-8-*-

import matplotlib.pyplot as plt
import numpy as np

# Kalman Filter implemented via Python
# Use a simple 6 * 6 CA model
class KalmanFilter:
    def __init__(self, state_dim = 2, obs_dim = 1):
        self.state_post     = np.zeros((state_dim, 1))
        self.state_pre      = np.zeros((state_dim, 1))      # 先验状态
        self.pre_cov        = np.eye(state_dim)             # 先验协方差
        self.post_cov       = np.eye(state_dim)             # 后验协方差
        self.trans_n        = np.eye(state_dim)             # 转移误差

        self.measure_n      = np.eye(obs_dim)             # 后验误差 
        self.measure_m      = np.eye(obs_dim)             # 观测转移矩阵（线性系统Kalman不会改变）
        self.state_dim      = state_dim 

    def setup(self, state_pre: np.ndarray, trans_n: np.ndarray = None, measure_m: np.ndarray = None):
        self.state_pre = state_pre
        self.state_post = state_pre
        if trans_n is not None:
            self.trans_n = trans_n
        if measure_m is not None:
            self.measure_m = measure_m

    @staticmethod
    def transition(dt):
        return np.array([
            [1, 0, dt, 0, 0.5 * dt ** 2, 0],
            [0, 1, 0, dt, 0, 0.5 * dt ** 2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]    
        ])

    def predict(self, trans:np.ndarray):
        self.state_pre = trans.dot(self.state_post)
        # 计算先验协方差 P(pre) = A * P(last post) * At + R(状态转移噪声协方差)
        self.pre_cov = trans @ self.post_cov @ trans.T + self.trans_n

    def correct(self, measure: np.ndarray, measure_m: np.ndarray):
        # 测量的总先验协方差：状态的先验协方差为P时，则 C * P(pre) * Ct + Q, Q为测量噪声大小
        tmp = measure_m @ self.pre_cov @ measure_m.T + self.measure_n

        # 计算Kalman增益 K(Gain) = P(pre) * Ct * 测量先验协方差的逆
        gain = self.pre_cov @ measure_m.T @ np.linalg.inv(tmp)

        # 后验（信息融合：控制与测量）的高斯均值： x(post) = x(pre) + K * (z - C * x(pre))
        self.state_post = self.state_pre + gain @ (measure - measure_m @ self.state_pre)

        # 转移后验协方差 (I - K * C) * P(pre)
        self.post_cov = (np.eye(self.state_dim) - gain @ self.measure_m) @ self.pre_cov

# =============================================
STATE_TRANS1 = np.array([
    [1, 0],
    [2, 0]
])
MEAS_TRANS1 = np.array([[1, 1]])
INIT_STATE1 = np.array([[0.2], [0.5]])
# =============================================
STATE_TRANS2 = np.array([
    [1, 0.5],
    [0, 1]
])
MEAS_TRANS2 = np.array([[1, 0]])
INIT_STATE2 = np.array([[0.0], [0.0]])
# =============================================
STATE_TRANS3 = np.array([
    [1, 2, 2],
    [0, 1, 2],
    [0, 0, 1]
])
MEAS_TRANS3 = np.array([[1, 0, 0]])
INIT_STATE3 = np.array([[0.0], [0.0], [0.2]])

def simple_experiments(
    state_trans: np.ndarray, 
    measure_trans: np.ndarray, 
    initial_state: np.ndarray, 
    var: list, state_dim = 2, obs_dim = 1, steps = 100, synthetic_plot = False
):
    np.random.seed(1)

    kf = KalmanFilter(state_dim, obs_dim)
    kf.setup(initial_state)
    kf.trans_n *= var[0]           # var = 0.1
    kf.measure_n *= var[1]

    gt_pos      = np.zeros((state_dim, steps))
    pred_pos    = np.zeros((state_dim, steps))
    variances   = np.zeros((state_dim, steps))
    for i in range(steps):
        w = np.random.normal(0, var[0], (1))
        v = np.random.normal(0, var[1], (1))
        gt_pos[:, i:i+1] = state_trans @ initial_state + w * np.ones((state_dim, 1))
        initial_state = gt_pos[:, i:i+1]

        kf.predict(state_trans)
        measure = measure_trans @ gt_pos[:, i:i+1] + v
        kf.correct(measure, measure_trans)

        pred_pos[:, i:i+1] = kf.state_post
        variances[:, i] = kf.post_cov.diagonal()
    
    print("KF completed\n")

    dims, seq_len = gt_pos.shape
    xs = np.arange(seq_len)
    plt.figure(1)
    for i in range(dims):
        plt.subplot(dims, 1, 1 + i)
        plt.plot(xs, variances[i, :], color = '#007CDD', label = 'variance', alpha = 0.8)
        plt.scatter(xs, variances[i, :], color = '#007CDD', s = 7)
        plt.grid(axis = 'both')
        plt.ylabel('Variance')
        plt.legend()
    plt.xlabel('Time steps')

    plt.figure(2)
    if synthetic_plot:
        plt.plot(gt_pos[0, :], gt_pos[1, :], c = 'r', label = 'ground truth', alpha = 0.8)
        plt.plot(pred_pos[0, :], pred_pos[1, :], c = 'b', label = 'KF prediction', alpha = 0.8)
        plt.scatter(gt_pos[0, :], gt_pos[1, :], color = 'r', s = 7)
        plt.scatter(pred_pos[0, :], pred_pos[1, :], color = 'b', s = 7)
        plt.grid(axis = 'both')
        plt.legend()
    else:
        plt.figure(2)
        for i in range(dims):
            plt.subplot(dims, 1, 1 + i)
            plt.plot(xs, gt_pos[i, :], color = '#009EFF', label = 'ground truth', alpha = 0.7)
            plt.plot(xs, pred_pos[i, :], color = '#E97777', label = 'KF prediction', alpha = 0.7)
            plt.scatter(xs, gt_pos[i, :], color = '#009EFF', s = 7)
            plt.scatter(xs, pred_pos[i, :], color = '#E97777', s = 7)
            plt.ylabel('Values')
            plt.grid(axis = 'both')
            plt.legend()
        plt.xlabel('Time steps')
        
    plt.show()

if __name__ == "__main__":
    from sys import argv
    exp_num = int(argv[1]) if len(argv) > 1 else 1
    if exp_num == 0:
        simple_experiments(STATE_TRANS1, MEAS_TRANS1, INIT_STATE1, [0.1, 0.1])
    elif exp_num == 1:
        simple_experiments(STATE_TRANS2, MEAS_TRANS2, INIT_STATE2, [1, 2.25])
    else:
        simple_experiments(STATE_TRANS3, MEAS_TRANS3, INIT_STATE3, [0, 0.15], 3, 1)

    

