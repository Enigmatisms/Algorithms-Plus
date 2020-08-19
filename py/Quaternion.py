#!/usr/bin/env python3
#-*-coding:utf-8-*-
# 四元数公式 验证四元数公式的正确性(欧拉角转四元数)
# 欧拉角转四元数的公式是无误的,错误的位置应该是坐标系的变换
# 注意到欧拉角转四元数 RPY 对应 XYZ轴
# 而在相机坐标系下, RPY对应 ZXY轴

# 个人认为的结果:
## XYZ转为ZXY
## 如果ZXY坐标系要旋转Pitch a 度, 相当于XYZ坐标系先绕roll转 a - 90 度 
## ZXY转yaw b 度, 则 XYZ 绕roll转 -90 再 绕

import numpy as np
import mpl_toolkits.mplot3d as mp3
import matplotlib.pyplot as plt

cosf = np.cos
sinf = np.sin

class Quaternion:
    def __init__(self, w = 0, x = 0, y = 0, z = 1):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def euler2Quat(self, r, p, y, is_rad = False):
        if is_rad == False:
            r *= 0.0174533
            p *= 0.0174533
            y *= 0.0174533     
        self.w = cosf(r / 2) * cosf(p / 2) * cosf(y / 2) + sinf(r / 2) * sinf(p / 2) * sinf(y / 2)
        self.x = sinf(r / 2) * cosf(p / 2) * cosf(y / 2) - cosf(r / 2) * sinf(p / 2) * sinf(y / 2)
        self.y = cosf(r / 2) * sinf(p / 2) * cosf(y / 2) + sinf(r / 2) * cosf(p / 2) * sinf(y / 2)
        self.z = cosf(r / 2) * cosf(p / 2) * sinf(y / 2) - sinf(r / 2) * sinf(p / 2) * cosf(y / 2)

    # 重载乘法
    def __mul__(self, other):
        out = Quaternion(0, 0, 0, 0)
        out.w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z;
        out.x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y;
        out.y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x;
        out.z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w;
        return out
    
    def __repr__(self):
        return str(self.w) + ", " + str(self.x) + ", " + str(self.y) + ", " + str(self.z)

    def __truediv__(self, n):
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def norm(self):
        return np.sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)

    def conj(self):
        return Quaternion(self.w, - self.x, - self.y, - self.z)

    def conjInPlace(self):
        self.x *= -1
        self.y *= -1
        self.z *= -1

    def normalize(self):
        n = self.norm()
        self.w /= n
        self.x /= n
        self.y /= n
        self.z /= n

    def normalized(self):
        n = self.norm()
        if n <= 0.000001:
            return Quaternion(0, 0, 0, 0)
        else:
            return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def inv(self):
        return self.conj() / (self.norm() ** 2)

    def invInPlace(self):
        self.conjInPlace()
        n2 = self.norm() ** 2
        self.w /= n2
        self.x /= n2
        self.y /= n2
        self.z /= n2

    def getSelf(self):
        return Quaternion(self.w, self.x, self.y, self.z)

    def rotate(self, x, y, z):
        dot = Quaternion(0, x, y, z)
        temp = self.getSelf() * dot * self.inv()
        return np.array([temp.x, temp.y, temp.z])

if __name__ == "__main__":
    x = np.array([1, -1, -1, 1])
    z = np.array([1, 1, -1, -1])
    y = np.linspace(0, 5, 20)
    xs = np.row_stack(tuple(x for i in range(20)))
    ys = np.column_stack(tuple(y for i in range(4)))
    zs = np.row_stack(tuple(z for i in range(20)))

    zeros = np.zeros((20, 1))
    axis = np.linspace(-8, 8, 40)

    xs = xs.ravel()
    ys = ys.ravel()
    zs = zs.ravel()

    fig = plt.figure()
    ax = mp3.Axes3D(fig)
    ax.scatter3D(xs, ys, zs)

    fig2 = plt.figure(2)
    ax2 = mp3.Axes3D(fig2)

    qt = Quaternion()

    # R P Y
    # 相机坐标系下 XYZ->ZXY
    # 则旋转pitch需要依靠x轴(正转)
    # 旋转yaw需要依靠y轴(逆转)
    qt.euler2Quat(0, -10, 0)

    dots = np.array([0, 0, 0])
    for dot in zip(xs, ys, zs):
        res = qt.rotate(*dot)
        dots = np.column_stack((dots, res))

    ax2.scatter3D(dots[0, 1:], dots[1, 1:], dots[2, 1:])

    ax.set_xlim([-4, 4])
    ax.set_ylim([-2, 6])
    ax.set_zlim([-4, 4])

    # ax.yaxis.set_ticks_position("top")
    # ax.invert_yaxis()

    ax2.set_xlim([-4, 4])
    ax2.set_ylim([-2, 6])
    ax2.set_zlim([-4, 4])

    qt2 = Quaternion()
    qt2.euler2Quat(-90, 0, 0)
    pos = (0, 1, 1)

    res = qt2.rotate(*pos)

    print("qt2 res")
    print(res)
    

    plt.show()
