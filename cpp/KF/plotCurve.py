import numpy as np
import matplotlib.pyplot as plt
from sys import argv

if __name__ == "__main__":
    if len(argv) < 2:
        print('Usage: python3 ./plotCurve.py <Analysis Error, 0 means plot curve only, 1 means error analysis>')
        exit(-1)
    data_r = np.loadtxt("./data/data_robust.txt", delimiter=',')
    data_s = np.loadtxt("./data/data_standard.txt", delimiter=',')
    xsr = np.arange(data_r.shape[0])
    xss = np.arange(data_s.shape[0])
    if int(argv[1]) == 0:
        plt.subplot(2, 1, 1)
        plt.title('Robust Kalman Filter')
        plt.scatter(xsr, data_r[:, 0], c = 'r', s = 7)
        plt.scatter(xsr, data_r[:, 1], c = 'b', s = 7)
    
        plt.plot(xsr, data_r[:, 0], c = 'r', label = 'Actual')
        plt.plot(xsr, data_r[:, 1], c = 'b', label = 'Predict')
        plt.grid(axis = 'both')
        plt.legend()
    
        plt.subplot(2, 1, 2)
        plt.title('Standard Kalman Filter')
        plt.scatter(xss, data_s[:, 0], c = 'r', s = 7)
        plt.scatter(xss, data_s[:, 1], c = 'b', s = 7)
    
        plt.plot(xss, data_s[:, 0], c = 'r', label = 'Actual')
        plt.plot(xss, data_s[:, 1], c = 'b', label = 'Predict')
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