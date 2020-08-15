#!/usr/bin/env python3
#-*-coding:utf-8-*-
# 四元数公式 验证四元数公式的正确性(欧拉角转四元数)

import numpy as np

class Quaternion:
    def __init__(self, w, x, y, z):
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
        if n <= 0.000001:
            self.w = 0
            self.x = 0
            self.y = 0
            self.z = 0
        else:
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
        return self.getSelf() * dot * self.inv()