# Algorithms-Plus
## Implementations of some important or interesting algos.
---
### TODOS
- [ ] 写一个shell语法总结性的文件
- [x] sklearn 学一下
- [x] sklearn PCA 数字识别
- [x] sklearn KNN 原理，自己的实现以及PCA KNN组合的机器学习

#### 数据处理 & 机器学习
- [ ] SVM learn / practice / application
- [ ] LDA: 暂无详细计划

#### 概率机器人
- [ ] 机器人运动分布
- [ ] 粒子滤波实现

#### 其他有意思的算法
- [ ] 蚁群算法(TSP问题分析)
- [ ] 退火算法(需要寻找实例)
- [ ] 遗传算法(需要寻找实例)

#### 数据结构与算法复习
- [ ] 栈的STL库使用
- [ ] 队列(queue)基操 deque对应函数的使用 priority_queue的应用
- [ ] 堆
- [ ] KD树
 
#### Recent
- [ ] CUDA Programming learning
- [ ] Buy a book related to CUDA GPU programming.

---
### 算法实现练习
* 2020.3.6
    * 实现了快速幂算法
    * 快速幂算法：可以在O(logN)事件内计算出a**N的结果，使用的是二进制原理，将指数用二进制位表示，对应二进制位为1时（使用与运算进行判断），就乘以目前的底数，如果为0则跳过， 此后底数进行平方运算，指数右移，知道指数右移后的结果为0
    * 跟进：快速幂取模运算
* 2020.3.6
    * 实现了快速幂取模算法
    * 实现了warshall算法
* 2020.3.11
    * 实现了垃圾欧几里得辗转相除法
    * 实现了Stein二进制位移gcd算法
* 2020.6.25
    * 树莓派 commit 测试
* 2020.8.15
    * 按照电控处所写的算法实现Quaternion,尝试使用这个四元数配合mpl_toolkits验证自己是否写对
---
### Shell 学习
* 2020.6.26
    * shell脚本练习第二次
* 2020.7.1
    * 完成了GitHub Push助手shell小项目
* 2020.07.07 
    * 编写完了 github README 助手，但受到shell语法限制
   * shell确实很多语法坑，比如什么时候要用变量引用，什么时候双引号
    * 不再处理基础shell相关内容
    * shell & C++(ofstream) 练习
* 2020.7.29
    * 写了两个实用脚本:
   * deskew.sh 用于消除文件传递导致的 cmake 时钟误差
   * replace.sh 用于快速替换文件转移时的路径
---
### 数据处理 & 优化 & 机器学习(非深度向)
* 2020.4.28
    * Eigen 入门（添加了eigenTest）
* 2020.07.15
    * Ceres库基本操作（一个二次函数的拟合）
* 2020.07.16
    * 写完了RANSAC拟合直线的算法(见py/ransac.py)
    * 废弃了gitPush.sh 原来的实现（原来的实现没有办法递归地求文件夹下已经修改的文件）
* 2020.07.22
    * PCA 理论到实践
* 2020.8.5
    * PCA KNN 用sklearn做出来了，但是自己写的PCA出错了
    * 个人认为问题在于： 对去中心化的过程理解不是很透彻，到底如何减去均值，为什么要这样减，什么是feature没有很好的理解
* 2020.8.11
    * PCA结束 对应的总结写在了 `Note/PCA.md`中
* 2020.8.12
    * SVM in sklearn, 还不太清楚classifier的几个参数的具体作用
    * 看了一下原理，后半部分暂时无法接受
* 2020.8.14
    * SVM 学习 (调用 SVC)
    * mpl_toolkits 以及 numpy 学习
---
### 机器视觉 & 滤波器
* 2020.5.14 凌晨
    * 创建了滤波算法集合 filter.py, 实现了限幅滤波以及中值平均滤波（未验证）
* 2020.5.x
    * ERAKF 论文`于当前加速度模型的抗差自适应Kalman滤波`的简单复现
* 2020.5.17 凌晨
    * 实现了多个滤波器，包括滑动平均，一阶滞后，限幅平均等
    * 全部通过了测试，并且想到了一个问题：如何把信号与系统学的知识运用到数字滤波器的设计上来？（由一阶滞后滤波器想到的，先要去推一下相关理论）
* 2020.07.08 
    * 做完了双目摄像头联合标定，可以用于计算四个矩阵
    * 实现了一个《概率机器人》上的运动模型，并且看了一下其采样分布
* 2020.7.27
    * 在python下实现了KF（作为KF实现复习以及numpy项目）
    * 对装甲板滑动平均进行了曲线验证
* 2020.8.15
    * 摩擦轮速度与裁判系统测速的一维Kalman in C
    * 双向链表实现的中值滤波 in C
* 2020.8.22
    * 动态规划初识：完成了三个背包问题，思路逐渐清晰
* 2020.8.24
    * 动态规划加强：在`py/DP/dp2.py`中实现了最大分割问题(递归，朴素非递归，内存优化非递归)
    * 在`py/DP/dp3.py`中实现了**恰好装满背包的可行性判定以及最大化价值**问题
    * 还剩余一个等和分割（来自leetcode）问题，此后就可以对着stixel world论文再体会一边DP的求解 
* 2020.8.26
    * 算法实现能力还是有待提高，实现了匈牙利算法，花了2个多小时才想明白
    * 这个匈牙利算法还不是非递归的，尝试了非递归的实现，发现比较恶心
    * 这个实现与匈牙利算法本身的说明（参见《离散数学》）非常不一样，书上说的方法很简单易懂，实现起来很难
    * 上午将DP算法的强化做完了，`py/DP/dp3.py`的算法存在很大漏洞，在`py/DP/dp4.py`中进行了阐述
    * 结合论文再看了一遍论文中的DP以及LUT生成
---
### CUDA GPU Programming
* 2020.8.19
    * CUDA: the first GPU program.
---
### 其他学习
* 2020.8.19
    * 第一篇论文：Stixel world GPU加速理论，读完并能顺利跑GitHub代码
    * 关于本篇论文的有关总结写在: `Paper & Summary / Paper / GPU-stixel-paper.md`
* 2020.8.20
    * 立体视觉入门PPT
    * 总结写在 `Paper & Summary / Stereo Vision Intros.md`
* 2020.8.21 - 22
    * MRF 以及概率论相关知识的补充
* 2020.8.23
    * 第二篇论文：`基于置信传播的双目匹配算法` 完成
    * 总结在 `Paper & Summary / Paper / Stereo Matching paper.md`

