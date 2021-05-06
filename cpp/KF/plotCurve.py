import numpy as np
import matplotlib.pyplot as plt
from sys import argv

if __name__ == "__main__":
    if len(argv) < 2:
        print('Usage: python3 ./plotCurve.py <Analysis Error, 0 means plot curve only, 1 means error analysis>')
        exit(-1)
    data_r = np.loadtxt("./data/data_robust.txt", delimiter=',')
    data_s = np.loadtxt("./data/data_standard.txt", delimiter=',')
    data_r = data_r[20:, :]
    xsr = np.arange(data_r.shape[0])
    xss = np.arange(data_s.shape[0])
    if int(argv[1]) == 0:
        plt.figure(0)
        plt.subplot(2, 1, 1)
        plt.title('Robust Kalman Filter X')
        plt.scatter(xsr, data_r[:, 0], c = 'r', s = 7)
        plt.scatter(xsr, data_r[:, 2], c = 'b', s = 7)
    
        plt.plot(xsr, data_r[:, 0], c = 'r', label = 'Actual')
        plt.plot(xsr, data_r[:, 2], c = 'b', label = 'Predict')
        plt.plot(xsr, data_r[:, 2] - data_r[:, 0], c = 'k', alpha = 0.6, linestyle = '--', label = 'difference')
        plt.grid(axis = 'both')
        plt.legend()
    
        plt.subplot(2, 1, 2)
        plt.title('Robust Kalman Filter Z')
        plt.scatter(xsr, data_r[:, 1], c = 'r', s = 7)
        plt.scatter(xsr, data_r[:, 3], c = 'b', s = 7)
    
        plt.plot(xsr, data_r[:, 1], c = 'r', label = 'Actual')
        plt.plot(xsr, data_r[:, 3], c = 'b', label = 'Predict')
        plt.grid(axis = 'both')

        plt.figure(1)
        plt.subplot(2, 1, 1)
        plt.title('X axis speed & acceleration')
        plt.scatter(xsr, data_r[:, 4], c = 'r', s = 7)
        plt.plot(xsr, data_r[:, 4], c = 'r', label = 'speed')
        plt.scatter(xsr, data_r[:, 6], c = 'b', s = 7)
        plt.plot(xsr, data_r[:, 6], c = 'b', label = 'acceleration')
        plt.legend()
        plt.grid(axis = 'both')

        plt.subplot(2, 1, 2)
        plt.title('X axis speed & acceleration & predict')
        plt.scatter(xsr, data_r[:, 4], c = 'r', s = 7)
        plt.scatter(xsr, data_r[:, 6], c = 'b', s = 7)
        spd = data_r[:, 4].copy()
        acc = data_r[:, 6].copy()
        spd = spd / np.max(spd) * np.max(data_r[:, 2])
        acc = acc / np.max(acc) * np.max(data_r[:, 2])
        plt.plot(xsr, spd, c = 'r', label = 'speed')
        plt.plot(xsr, acc, c = 'b', label = 'acceleration')
        plt.plot(xsr, data_r[:, 2], c = 'k', label = 'prediction', linestyle='--', alpha = 0.5)
        plt.plot(xsr, data_r[:, 0], c = 'g', label = 'actual', linestyle='-.', alpha = 0.8)
        plt.grid(axis = 'both')
    else:
        err_r = data_r[:, 0] - data_r[:, 1]
        err_s = data_s[:, 0] - data_s[:, 1]

        plt.subplot(2, 1, 1)
        plt.title('Robust Kalman Filter')
        plt.scatter(xsr, err_r, c = 'r', s = 7)
        plt.plot(xsr, err_r, c = 'r')
        plt.grid(axis = 'both')

        plt.subplot(2, 1, 2)
        plt.title('Standard Kalman Filter')
        plt.scatter(xss, err_s, c = 'b', s = 7)
        plt.plot(xss, err_s, c = 'b')
        plt.grid(axis = 'both')
        
    plt.legend()
    plt.show()