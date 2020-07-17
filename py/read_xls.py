#-*-coding:utf-8-*-

import xlrd
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    data = xlrd.open_workbook('/home/sentinel/sm/dd.xlsx')
    table = data.sheets() [0]
    xs_raw = table.col(0)
    ys_raw = table.col(1)
    
    xs = np.array([x.value for x in xs_raw])
    ys = np.array([y.value for y in ys_raw])

    coe = np.polyfit(xs, ys, 12)
    fn = np.poly1d(coe)

    ys_ = np.array([ fn(x) for x in xs])


    # 0 - 0.6

    thresh = 0.1
    pts = [(xs[i], ys[i]) for i in range(len(xs))]
    total = len(pts)
    b_var = 0
    ideal_th = 0.1
    while(thresh <= 0.8):
        up_pt = []
        low_pt = []
        for pt in pts:
            if pt[1] > thresh:
                up_pt.append(pt)
            else:
                low_pt.append(pt)
        mean1 = np.mean( np.array([pt[1] for pt in up_pt]) )
        mean2 = np.mean( np.array([pt[1] for pt in low_pt]) )
        var = len(up_pt) * len(low_pt) / (total ** 2) * (mean1 - mean2) ** 2
        if var > b_var:
            ideal_th = thresh
            b_var = var
        thresh += 0.01

    print(ideal_th)

    ideal_th = 0
    xs_ideal = []
    ys_ideal = []
    i = 0
    for i in range(total):
        if pts[i][1] > ideal_th:
            break

    for k in range(i):
        xs_ideal.append(pts[k][0])
        ys_ideal.append(pts[k][1])

    for k in range(i, total):
        if pts[k][1] > ideal_th and pts[k][0] <= 1040:
            xs_ideal.append(pts[k][0])
            ys_ideal.append(pts[k][1])
    
    fn_coe = np.polyfit(xs_ideal, ys_ideal, 4)
    fn = np.poly1d( fn_coe )

    print("Maximum:", xs_ideal[-1])
    print(fn_coe)
    print(fn)

    sub1 = plt.subplot(211)
    plt.scatter(xs, ys, c = "black", marker = "o", label = "data", s = 1)
    plt.plot(xs, ys_, color = "red", label = "12-order polyfit")
    plt.plot([0, 3000], [ideal_th, ideal_th], color = "blue", label = "ideal threshold")

    sub2 = plt.subplot(212)
    plt.scatter(xs_ideal, ys_ideal, c = "black", marker = "o", label = "ideal pts", s = 1)
    plt.plot(xs_ideal, [fn(x) for x in xs_ideal], color = "red", label = "OTSU result")
    sub1.legend()
    sub2.legend()

    sub1.set_title("Whole dataset without threshold")
    sub2.set_title("OTSU threshold result / polyfit (3-order)")
    
    plt.show()
