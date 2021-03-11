#-*-coding:utf-8-*-

import cv2
import numpy as np
import os
import sys

SZ = 32
cam_width = 640
cam_height = 480
record_fps = 60
disable_test_set = False    # 训练是否加载测试集，如果为False，每次训练结束后要在data文件夹下的'v'开头文件夹放入少量用于验证的图片

# ============== 状态标志 ===============
RECORD_SAVE_PICS = 0        # 不进行录像，保存训练数据为图片，到文件夹data下
VIDEO_SAVE_PICS = 1         # 录像，录像结束之后（esc）从录制的视频中读取，输出图片
TRAIN_WITH_PICS = 2         # 手动对data文件夹下的数据进行分类，放到对应文件夹后进行训练
USING_SVM_MODEL = 3         # 使用训练好的模型进行检测

class Detector:
    colors = (
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (0, 255, 255)
    )

    label_text = (
        'pinchers',
        'milk',
        'tape',
        'bottle'
    )
    def __init__(self, use_load = False):
        self.model = cv2.ml.SVM_create()
        
        if use_load:
            self.model = cv2.ml.SVM_load("./svm.xml")
        else:
            self.model.setType(cv2.ml.SVM_C_SVC)
            self.model.setKernel(cv2.ml.SVM_RBF)
            self.model.setGamma(0.01)
            self.model.setC(10.0)
            self.model.setTermCriteria((cv2.TERM_CRITERIA_EPS, 1000, 1e-5))
        self.total_cnt = 1
            
    @staticmethod
    def loadPics(root = "./data/", test = False):
        data = None
        label = None
        init_data = False
        init_label = False
        for i in range(4):      # 只考虑4分类的情况
            if test == False:
                cur_dir = root + "%d/"%(i)
            else:
                cur_dir = root + "v%d/"%(i)
            cnt = 0
            print("Accessing directory: ", cur_dir)
            while True:
                path = cur_dir + "%d.jpg"%(cnt + 1)
                if os.path.exists(path) == False:
                    break
                img = cv2.imread(path, 0).astype(np.float32)
                img = cv2.normalize(img, img, alpha=1.0, beta=0.0)
                if init_data == False:
                    init_data = True
                    data = img.copy().ravel()
                else:
                    data = np.vstack((data.copy(), img.ravel()))
                cnt += 1
            if init_label == False:
                init_label = True
                label = np.ones((cnt, 1)) * i
            else:
                label = np.concatenate((label, np.ones((cnt, 1)) * i))
        print("Data loaded: ", data.shape, ", label: ", label.ravel())
        return data, label

    # X输入 n * (SZ * SZ)的数据集，n为样本数量
    def fit(self, X:np.ndarray, y:np.ndarray):
        print("SVM start to train..., X shape: ", X.shape, ", y shape: ", y.shape)
        if not X.dtype == np.float32:
            X = X.astype(np.float32)
        if not y.dtype == np.int32:
            y = y.astype(np.int32) 
        result = self.model.train(X, cv2.ml.ROW_SAMPLE, y)
        print("Training complete: ", result)

    def predict(self, X):
        if not X.dtype == np.float32:
            X = X.astype(np.float32)
        return self.model.predict(X)

    def saveModel(self, opath = "./svm.xml"):
        self.model.save(opath)


    # AffineTransform会导致严重的变形
    def preProcessNonAffine(self, img:np.array, save = False):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, threshed = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)
        threshed = cv2.bitwise_not(threshed)
        k1 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        k2 = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        threshed = cv2.erode(threshed, k1)
        threshed = cv2.dilate(threshed, k2, iterations = 5)
        cv2.imshow('th', threshed)
        contours, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        boxes = []
        for c in contours:
            box = cv2.boundingRect(c)
            if box[2] * box[3] > 4900:
                boxes.append(box)
        results = []
        for box in boxes:
            rstart = box[1]
            rend = box[1] + box[3]
            cstart = box[0]
            cend = box[2] + box[0]
            cropped = gray[rstart:rend, cstart:cend]
            rsz = cv2.resize(cropped, (SZ, SZ))
            if save:
                cv2.imwrite("./data/%d.jpg"%(self.total_cnt), rsz)
                self.total_cnt += 1
            rsz = rsz.astype(np.float32)
            rsz = cv2.normalize(rsz, rsz, alpha = 1.0, beta = 0.0)
            results.append(rsz.ravel())
        if len(results) == 0:
            return np.array(None), boxes
        return np.vstack(results), boxes

    @staticmethod
    def drawPredicted(src, labels, boxes):
        font = cv2.FONT_HERSHEY_SIMPLEX
        for label, box in zip(labels, boxes):
            label = int(label)
            color = Detector.colors[label]
            pt1 = tuple(np.array([box[0], box[1]]))
            pt2 = tuple(pt1 + np.array([box[2], box[3]]))
            cv2.rectangle(src, pt1, pt2, color, 2)
            cv2.putText(src, Detector.label_text[label], pt1, font, 1, color, 2)
        return src

# 测试SVM分类器的效果
def trainSVMTest(test = True):
    clf = Detector(use_load = False)
    train_data, train_label = Detector.loadPics()
    if test == True:
        test_data, test_label = Detector.loadPics(test = True)
    clf.fit(train_data, train_label)

    if test == True:
        result = clf.predict(test_data)
        print("Test labels are: ", test_label.ravel())
        print("Predict labels are: ", result[1].ravel())

    clf.saveModel()

# 测试camera 是否可以正常打开
def cameraFeed(cam_idx, use_record = False, output_path = "./record.avi"):
    cap = cv2.VideoCapture(0)
    cap.open(cam_idx)
    cv2.namedWindow('disp', cv2.WINDOW_AUTOSIZE)
    shape_known = False
    recording = False
    if use_record == True:
        sz = (cam_width, cam_height)
        wri = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc('I', '4', '2', '0'), record_fps, sz)
    while(True):
        _, img = cap.read()
        height, width, _ = img.shape
        if shape_known == False:
            shape_known = True
        if width == 0 or height == 0:
            break
        if use_record and recording:
            wri.write(img)
            cv2.circle(img, (24, 24), 10, (0, 0, 255), -1)
        cv2.imshow('disp', img)
        k = cv2.waitKey(1)
        if k == 27:
            break
        elif k == ord(' '):
            cv2.waitKey(0)
        elif k == ord('e'):
            recording = not recording
    cv2.destroyAllWindows()
    if use_record == True:
        print("Recording completed, output file saved to '%s'"%(output_path))
        wri.release()
    cap.release()

# 开启图像记录 分割图片，输出到data文件夹下
# 需要人工对data文件分类
def dataAcquisition(use_cam = True, file_path = "./record.avi", use_predict = False):
    cap = cv2.VideoCapture()
    if use_cam:
        cap.open(2)
    else:
        cap.open(file_path)
    dt = Detector(True)
    while True:
        success, img = cap.read()
        if success == False or img is None: break
        segs, boxes = dt.preProcessNonAffine(img, True)
        if len(boxes):
            _, labels = dt.predict(segs)
            img = Detector.drawPredicted(img, labels, boxes)
        cv2.imshow('disp', img)
        k = cv2.waitKey(10)
        if k == 27:
            break
        elif k == ord(' '):
            cv2.waitKey(0)
    cap.release()

if __name__ == "__main__":
    # 参数设置
    index = 2                       # 摄像头索引
    status = RECORD_SAVE_PICS       # 见文件顶部
    test_after_train = True         # 加载测试集

    if len(sys.argv) < 2:
        print("Usage: python3 ./threshed.py -status")
    else:
        status = int(sys.argv[1])
    if status == RECORD_SAVE_PICS:
        dataAcquisition(True, save_pic = True)
    elif status == VIDEO_SAVE_PICS:
        cameraFeed(index, True)
        dataAcquisition(False, save_pic = True)
    elif status == TRAIN_WITH_PICS:
        trainSVMTest(test_after_train)
    elif status == USING_SVM_MODEL:
        cap = cv2.VideoCapture()
        cap.open(index)
        dt = Detector(True)
        while True:
            success, img = cap.read()
            if success == False or img is None: break
            segs, boxes = dt.preProcessNonAffine(img, False)
            if len(boxes):
                _, labels = dt.predict(segs)
                # ============================ 使用 label 的代码加在此处（比如要将label发出去） ===========================
    
                # ===================================================================================================
                img = Detector.drawPredicted(img, labels, boxes)
            cv2.imshow('disp', img)
            k = cv2.waitKey(10)
            if k == 27:
                break
            elif k == ord(' '):
                cv2.waitKey(0)
        cap.release()
    else:
        print("Status doesn't exist.")