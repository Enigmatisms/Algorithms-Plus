# 论文总结
## GPU-accelerated real-time stixel computation

---
### 纯理论理解

#### 求解双目图像stixel步骤
1. 取视差图，视差图可能为作为计算结果之前就存在于显存中
2. 行分块，假设长度为s的像素为一组，原来的w*h（像素）将被重新划分为(w/s * h)
3. 每行中的新单元：长度为s的像素块 求取平均视差作为此像素块的视差(disparity)
4. 像素块分类`Page 4`
    > * 视差几乎为0的点为极远点，为`sky`类
    > * 视差与地面点存在某一梯度关系的点分类为`ground`
    > * 竖列上变化很小，并且满足特定约束的点为`obj`
5. 分类后对`sky`以及`gnd`类别的点进行预计算，存储于LUT(Look up table)中
6. 构建MAP问题并使用动态规划求解(DP)
   * 从pos 0开始向上进行递推
   * 在每一个新位置进行计算时，可以利用之前得到的结果，并衡量：新的位置是否于之前的segment是连接的？如果不是连接的，属于哪个类型的stixel最为合适？
   * 每一步都依赖之前的计算结果，但由于之前的计算结果被保存，无需继续计算
   * 保存的值储存在Cost table中，这个table为w / s * h大小的，对应于每一个`像素块`的权重以及分类情况
   * 动态规划的过程中同时计算Cost Table
7. 回溯法
   * 相当于对DP的分类计算结果进行重新组织
   * 给我的感觉好像是一个DFS 贪婪地寻找临近最小值点，组织在一起

#### Acceleration Strategy

* Pre Comupute method:
   * For pixel batches or stixels labelled `sky` or `gnd`, the cost will be pre-comupted.
   * Results of the former computation will be used recursively.
 * LUTs (Look up table):
   * Pre comuputed costs are stored in LUTS, which is basically an approach (Space -> Time)
   * LUTs are 3 dims with (h + 1) , w / s, d
 * Prefix sum operations:
   * Parallel optimization of sum calculation for arrays.
 * Column reduction and transpose:
   * w * h data -> w/s * h data -> transpose: (GPU will process data by row, while stixels are column-arranged.)
---
### Understanding with code
1. Get image and open up memory space in GPU (cudaMallocHost): copying the image from host memory to GPU.
2. Memcpy to GPU `Stixels::SetDisparityImage: cudaMemcpyAsync` the origin (no column reduction) image.
3. Precomputations:
   1. Camera parameters. `RoadEstimation::Compute` using a Hough transformation to get horizon, etc.
4. Stixels compute: `Compute`
   1. In intialization, sky is already pre computed. `PrecomputeSky`
   2. Disparity mean for each pixel batch: `JoinColumns`
   3. LUT calculation (Lookup table recomputation).  (A cuda function) `ComputeObjectLUT`
   4. A loooooooooonnnnnnnnngggggggg fucntion `StixelsKernel` (CUDA KernelFunction):
      1. First, calculate Cost of pixels labelled `sky` and `ground`
        * Notice that: skt_lut and gr_lut are `w / s * h` because they do not depend on input data.
        * Whereas obj_lut is `w / s * drange * h`, which is 3-dim
        * Notice that **Transpose operation** is used here, for computation takes place by row instead of "col"
      2. Prefix sum computation. `ComputePrefixSum` for fast Cost computation: 
      > Then, calculating the cost of a stixel is
      > done in constant time just by subtracting two numbers in
      > the table, independently of the size of the stixel. 
      3. Base case problem solving.
      4. **DP PROCESS**: notice that prior probability from obj to obj depends on the mean of the disparity of segment.
      5. Warps are decreasing as the operation goes.
      > the last active thread in a CTA is responsible of
      > generating the final output 
      6. Yield the result of backtracing.
      7. Stop recording and return.(before this, synchronization)
5. Yield Section (All the stixels)
6. Algorithm completed.
---
##### The problems I still have:
###### Code-free reading
- [x] What the hell is drange, why LUTs should be arranged in 3-dim array?
  * Possible mean disparity of the stixel.
  * Notice that: `Object pixel belongs to one certain stixel, but the cost depends one the mean disparity of the stixel.`
  * i.e. I don't know what kind of stixel one particular pixel shall belong to, therefore I have to list all of them.
  * The number of the possible stixel mean disparity is `drange`, and each one of the pixel has `drange` choices.
- [x] The real procedure of DP and how is the cost table derived?
  * Of course, using data recursively, only the last (newly added) batch shall be calculated.
  * Massive parallelism is used.
- [x] Pixel batches, how are they connected?
  * The paper doesn't use backtracing to connect pixel batches, pixel batches are originally connected.
  * During DP Process, batches will be classified into 3 classes and estimate the cost.
  * Then backtracing connect ***The result stixels.***
###### Along with code
- [ ] max disparity: what is this? How does this affect ground computation?
---
#### Thoughts about modification
- Since the algorithm is beyond what I can come up with, I have no thoughts about modifying the logic of the algorithm.
- Yet I found out that, those prior probability parameters are fixed, which is obviously optimal
- It is based on experience and fixed, which is not good.
- If we can use some sort of approach to modify the input prior probs, just like what KF does
- It would be fine, to use observation to optimize the prior probs.