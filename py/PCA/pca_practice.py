#!/usr/bin/env python3
#-*-coding:utf-8-*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mmg
import sklearn

def norm(array:np.array):
    res = np.zeros(array.shape)
    mean_res = np.array([ [np.mean(array[i, :])] for i in range(array.shape[0]) ])
    for i in range(array.shape[0]):
        res[i, :] = array[i, :]
    res -= mean_res
    # 注意行列向量
    return res, mean_res

if __name__ == "__main__":
    # 读入图片， 返回 numpy.array
    img = mmg.imread('./resources/lena.jpg')
    print("Pic shape:", img.shape)
    rows, cols, chs = img.shape
    # 先取每一行 每一行是(3 * cols)
    # 取所有行的第i(每行的第i列)个元素，对所有行

    sub1 = plt.subplot(231)
    plt.imshow(img)
    sub1.set_title("Original")

    rcs = np.array([img[i][:, 0] for i in range(rows)])     
    gcs = np.array([img[i][:, 1] for i in range(rows)])
    bcs = np.array([img[i][:, 2] for i in range(rows)])
    max_len = int(min(cols, rows) * 0.6)

    comp_pic = np.zeros((rows, cols, chs))
    channel_cnt = 0

    dim = max_len

    # 只取用左奇异值的PCA，为什么会这样呢
    # 三通道的使用写的复杂了些
    for mat in [rcs, gcs, bcs]:
        normed, means = norm(mat)
        cov = normed.dot(np.transpose(normed))

        u, s, v = np.linalg.svd(cov)

        vecs = u[:, :dim]       # 取前dim个特征向量(u的列表示特征向量， v的行表示特征向量, usr' = usv = svd(cov))
        vecs = np.transpose(vecs)
        # 则降维后的数据为:
        # 数据降维
        new_data = vecs.dot(mat)

        # 数据恢复（失真） 恢复的结果是float
        # 此处为 取部分 u设为 k, pca就是 k.transpose() * mat, 恢复应该是 k * k.transpose() * mat
        rebuild = np.transpose(vecs).dot(new_data)
        maximum = np.max(rebuild)
        rebuild *= (2.6 / maximum)
        rebuild = rebuild.astype(int)

        for i in range(rows):
            comp_pic[i, :, channel_cnt] = rebuild[i, :]
        channel_cnt += 1
    
    print("Compressing completed with mean:", np.mean(comp_pic))

    # svd 压缩方式
    sub2 = plt.subplot(232)
    plt.imshow(comp_pic)
    sub2.set_title("Direct PCA")

    com16 = np.zeros_like(img)
    com32 = np.zeros_like(img)
    com64 = np.zeros_like(img)
    com128 = np.zeros_like(img)
    lst = [com16, com32, com64, com128]
    for k in range(4):
        for i in range(3):
            u, s, v = np.linalg.svd(img[:, :, i])
            lst[k][:, :, i] = u[:, :2**(4 + k)].dot(np.diag(s[:2**(4 + k)])).dot(v[:2**(4 + k), :])

        sub = plt.subplot(233 + k)
        plt.imshow(lst[k])
        sub.set_title("SVD " + str(2**(4+k)) + " eig vectors")


    plt.subplots_adjust(left = 0.1, right= 0.9, top = 0.9, bottom = 0.1, hspace = 0.3, wspace = 0.3)

    plt.show()







    
        
        

    
    
        
