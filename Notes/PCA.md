# PCA Notes

## 2020.8.11 by Enigmatisms

### How to display PCA compression

> * For images, no decentralization is needed. Therefore, image data (Let it be A(M * N))

>> * Use simple SVD: `U, S, Vt = np.linalg.svd(A)`

>> * Notice that U (M * M), S (M * N) (yet the result from numpy is an 1-D array), Vt (N * N)

>> * To display the compressed image, U and Vt are both needed.

>>> * `U[:, :k].dot(np.diag(S[:k])).dot(Vt([:k, :]))`

>>> * Which means, extracting k cols from U, k rows from Vt, we shall get U'(M * K) * S(K * K) * Vt(K * N)

>> * Then we have a (M * N) matrix, literally compressed while in terms of the amount of data: not compressed.

### To compress Images

> * Only U or Vt is needed.

> * Extract k cols from U, k rows from Vt (equivalent to the k-cols extraction from V)

> * Through `U[:, :k].dot(S[:k, :k])` or `S[:k, :k].dot(Vt[:k, :])` we get M * K or K * N matrix.

### For other featured data

> * Images have no explicit feature, whereas for other kinds of data like:

|Stu.|Chinese|Math|Eniglish|
|:-:|:-:|:-:|:-:|
|1|80|90|82|
|2|76|73|93|
|3|81|40|52|
|4|67|66|78|

> * The data listed above is obviously featured. Therefore, we need decentralization.

#### What is 'Feature'

- Different properties within one sample, in this case: C, M, E scores.

- Respectively, `the students` are data.

#### Decentralization

> * For the example above, the samples and features are arranged in a row-col pattern

>> * We should get the mean value of each feature but not each sample, in this case we need mean values of columns.

>> * Deduction.

#### Covariance & Loss

> * To distinguish each one of the datum, we expect those n-dimension data dots to scatter as much as possible.

>> * Covariance is the barometer of 'How scattered the data are'

>> * We want to retain as much info as possible in the directions which variance is big (implying dispersity)

>> * Use sorted eigen values and eigen vectors

### My job

- [x] PCA learning, dimention reduction on 3-D data (down to 2-D) `pca_learn.py`

- [x] PCA pratice, picture compressing and displaying.  `pca_pratice.py`

- [x] PCA and KNN, using PCA to reduce the amount of data for KNN. `pca_knn.py` 

### Platform

- numpy

- matplotlib

- sklearn