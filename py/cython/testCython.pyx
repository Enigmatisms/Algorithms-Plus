cimport numpy as np
cimport cython
import numpy as np

@cython.boundscheck(False)
@cython.wraparound(False)
def annealing(np.ndarray[np.float64_t, ndim=1] seg, int end, int max_iter = 30, float ab_thresh = 0.03, float start_temp = 0.3):
    cdef int now_pos = end
    cdef int seg_len = len(seg)
    cdef int min_pos = now_pos
    cdef int tipping_cnt = 0
    cdef int i = 0
    cdef int step = 0
    cdef int tmp_pos = now_pos
    cdef float temp = 0.0
    cdef float this_val = 0.0
    cdef float min_val = seg[now_pos]
    while i < max_iter:
        temp = start_temp / (i + 1)
        if i > 6:
            step = np.random.choice((1, 2)) * np.random.choice((-1, 1))
        elif i > 2:
            step = np.random.choice((1, 2, 3)) * np.random.choice((-1, 1))
        else:
            step = np.random.choice((1, 1, 2, 2, 3))
        tmp_pos = now_pos + step
        if tmp_pos >= seg_len:
            tmp_pos = seg_len - 1
        elif tmp_pos <= 0:
            tmp_pos = 0
        now_val = seg[tmp_pos]
        if now_val < min_val:       # 接受当前移动以及最终移动
            min_val = now_val
            now_pos = tmp_pos
            min_pos = tmp_pos
        else:
            if  np.random.uniform(0, 1) < np.exp(- (now_val - min_val) / temp):
                now_pos = tmp_pos
        this_val = seg[now_pos]
        if this_val < ab_thresh:
            return min_pos
        elif this_val < 2 * ab_thresh:
            if np.random.uniform(0, 1) < np.exp( - tipping_cnt / 2):
                return min_pos
            tipping_cnt += 1
        i += 1
    return min_pos