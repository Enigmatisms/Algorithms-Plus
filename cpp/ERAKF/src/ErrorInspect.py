#!/usr/bin/env python
#-coding:utf-8-*-


# 比较两次预测补偿量
import numpy as np
import random
import matplotlib.pyplot as plt
import json

if __name__ == "__main__":
    with open("../py_standard.txt", "r") as rd:
        data = json.load(rd)
    with open("../pyf.txt", "r") as rd:
        kf_data = json.load(rd)

    with open("../data/test3/py_yaw_error.txt", "r") as rd:
        tmp = json.load(rd)
        old_eyaw = np.array(tmp)
    
    with open("../data/test3/py_pit_error.txt", "r") as rd:
        tmp = json.load(rd)
        old_epit = np.array(tmp)

    delayed = data[6:]                                # 完全无预测模型
    data = data[7:]
    last_e = data[len(data) - 1]
    data.append(last_e)

    len_diff = len(kf_data) - len(data)
    print("Kf data is %d longer than data"%len_diff)
    kf_data = kf_data[len_diff:]
    
    length = len(data)
        
    data_yaws = np.array([i[0] for i in data])        # 取出二维lst中每个分量的首位
    data_pics = np.array([i[1] for i in data])        # 取出二维lst中每个分量的末位

    delay_yaws = np.array([i[0] for i in delayed])      # 无预测量（一阶滞后）
    delay_pics = np.array([i[1] for i in delayed])      # 同上
    
    kf_yaws = np.array([i[0] for i in kf_data])       # 取出二维lst中每个分量的首位
    kf_pics = np.array([i[1] for i in kf_data])       # 取出二维lst中每个分量的末位
    xs = np.array([i for i in range(length)])
    zero = np.array([0 for i in range(length)])       # 零基准线

    # 计算预测实际补偿的量(感觉不大)
    error_yaws =  - delay_yaws + kf_yaws              #kf_yaws - data_yaws
    error_pics =  - delay_pics + kf_pics              #kf_pics - data_pics

    # comp_yaw = error_yaws - old_eyaw
    # comp_pit = error_pics - old_epit
    
    plt.subplots_adjust(hspace = 0.4)
    sub1 = plt.subplot(211)
    plt.plot(xs, zero, color = "green")
    plt.plot(xs, error_yaws, color = "red", label = "Classical KF")
    plt.plot(xs, old_eyaw, color = "blue", label = "ARKF")
    sub1.set_title("The yaw compensation of CKF and ARKF")

    sub2 = plt.subplot(212)
    plt.plot(xs, zero, color = "green")
    plt.plot(xs, error_pics, color = "red", label = "Classical KF")
    plt.plot(xs, old_epit, color = "blue", label = "ARKF")
    sub2.set_title("The pitch compensation of CKF and ARKF")

    sub1.legend()
    sub2.legend()
    plt.show()

    with open("../py_yaw_error.txt", "w") as wri:
        json.dump(list(error_yaws), wri)

    with open("../py_pit_error.txt", "w") as wri:
        json.dump(list(error_pics), wri)




    
    