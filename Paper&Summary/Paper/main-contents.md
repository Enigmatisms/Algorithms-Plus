## Algorithm-Plus 主要内容
---
### 逻辑性 / 技巧性算法
* 2020.3.6
    * 实现了快速幂算法
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
* 2020.8.28 / 29
    * 完成了遗传算法-MTSP问题
    * 完成了蚁群算法-TSP问题
* 2020.8.31
    * 完成了Metropolis-Hasting 算法 但总感觉还有哪里没有理解，说不上来，再看看吧
    * 完成了退火算法 一元函数优化分析
* 2020.9.15
    * PID
    * 变结构PID
* 2020.9.16
    * C++ 归并排序 （递归）（注意自己的写法）
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
* 2020.9.2
    * LDA 学习 (手写实现了LDA + 调库 尝试绘制分类图）
* 2020.9.9
    * 写了一个很简单的EM算法实例 （迭代分类的MLE---根据上一布判定本次样本最可能属于哪类）
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
* 2020.9.3
    * 双边滤波(bilateral filtering)
    * 实现的结果与opencv库一致，可以使用CUDA并行加速
* 2020.9.4
    * Eigen EKF 正弦变速圆的跟踪预测
* 2020.9.4
    * 尝试使用RANSAC对匹配进行优化，还没有完成
    * 要逐步转向感知优化方面，而不继续做控制优化
* 2020.9.5
    * 光流 反向法测试
    * 这个地方的某些（导数的方向，正负号选取）特别迷
* 2020.9.7
    * 直接法求相机位姿优化 Ceres 未完成
* 2020.9.8
    * 最后Ceres求解失败的原因很难找，其中存在一些非常奇怪的输出
    * 我在想是不是operator中的K，Kinv的问题
* 2020.9.9
    * 求解成功，对于KITTI数据集中的某两张图进行了测试
    * 实际上这个方法不太稳定，运动稍微大一些就会引起不收敛的情况
    * 不稳定程度：01.png 三层金字塔 边缘禁选区宽度10（改变任意一个值都会导致不收敛）
    * 在Ceres学习更加深入之后回来解决这个问题
    * SLAM14讲上的手写求解倒是可以用，不知道Ceres的问题具体在哪
    * 可以尝试用ceres写一个反向光流法

---
### CUDA GPU Programming
* 2020.8.19
    * CUDA: the first GPU program.
* 2020.8.27
    * CUDA 基础
* 2020.8.31
    * 实现了CUDA并行的 opencv 相关算法：阈值化以及灰度化
---
### 其他学习
- 总结在 `./Paper&Summary/` 下
* 2020.8.19
    * 第一篇论文：Stixel world GPU加速理论，读完并能顺利跑GitHub代码
    * 关于本篇论文的有关总结写在: [GPU-stixel-paper.md](https://github.com/Enigmatisms/Algorithms-Plus/blob/master/Paper%26Summary/Paper/GPU-stixel-paper.md)
* 2020.8.20
    * 立体视觉入门PPT
    * 总结写在 [StereoVisionIntros.md](https://github.com/Enigmatisms/Algorithms-Plus/blob/master/Paper%26Summary/StereoVisionIntros.md)
* 2020.8.21 - 22
    * MRF 以及概率论相关知识的补充
* 2020.8.23
    * 第二篇论文：`基于置信传播的双目匹配算法` 完成
    * 总结在 [Stereo-Matching-paper.md](https://github.com/Enigmatisms/Algorithms-Plus/blob/master/Paper%26Summary/Paper/StereoMatchingpaper.md)
* 2020.8.31
    * 第三篇论文`VLOAM`完成
    * 总结在 [VLOAM.md](https://github.com/Enigmatisms/Algorithms-Plus/blob/master/Paper%26Summary/Paper/VLOAM.md)
* 2020.9.2
    * 论文`SVO` 以及 `LK 20 years`(节选) 完成
    * 见 [SVO.md](https://github.com/Enigmatisms/Algorithms-Plus/blob/master/Paper%26Summary/Paper/SVO.md)
* 2020.9.3
    * 论文`SOFT-SLAM`完成
    * 见[SOFT-SLAM.md](https://github.com/Enigmatisms/Algorithms-Plus/blob/master/Paper%26Summary/Paper/SOFT-SLAM.md)
