# Algorithms-Plus
## Implementations of some important or interesting algos.

# 算法实现练习
> * 2020.3.6
>> * 实现了快速幂算法
>> * 快速幂算法：可以在O(logN)事件内计算出a**N的结果，使用的是二进制原理，将指数用二进制位表示，对应二进制位为1时（使用与运算进行判断），就乘以目前的底数，如果为0则跳过， 此后底数进行平方运算，指数右移，知道指数右移后的结果为0
>> * 跟进：快速幂取模运算
> * 2020.3.6
>> * 实现了快速幂取模算法
>> * 实现了warshall算法
> * 2020.3.11
>> * 实现了垃圾欧几里得辗转相除法
>> * 实现了Stein二进制位移gcd算法
>> * 个人感觉自己的思考还是少了，虽然看一眼算法思路就知道怎么实现，但是实现的时候还是笨手笨脚的，练的还是少了
> * 2020.4.28
>> * Eigen 入门（添加了eigenTest）
> * 2020.5.14 凌晨
>> * 创建了滤波算法集合 filter.py, 实现了限幅滤波以及中值平均滤波（未验证）
> * 2020.5.17 凌晨
>> * 实现了多个滤波器，包括滑动平均，一阶滞后，限幅平均等
>> * 全部通过了测试，并且想到了一个问题：如何把信号与系统学的知识运用到数字滤波器的设计上来？（由一阶滞后滤波器想到的，先要去推一下相关理论）
> * 2020.6.25
>> * 树莓派 commit 测试

> * 2020.6.26
>> * shell脚本练习第二次
> * 2020.7.1
>> * 完成了GitHub Push助手shell小项目
> * 2020.07.07 
>> * 编写完了 github README 助手，但受到shell语法限制
>>> * shell确实很多语法坑，比如什么时候要用变量引用，什么时候双引号
>> * 不再处理基础shell相关内容
- [ ] 写一个shell语法总结性的文件
