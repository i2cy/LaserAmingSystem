#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: BoranLi(https://github.com/Spc2r)
# Contributor: I2cy(i2cy@outlook.com)

import threading
import time
import cv2
import ctypes
from multiprocessing import Process, Value, Queue


class CameraPipe:

    def __init__(self, video_capture_args, frame_size, frame_rate=60):
        self.__video_args = video_capture_args
        self.frame_size = frame_size
        self.frame_rate = frame_rate
        self.__queue_frame = Queue(128)
        self.__live = Value(ctypes.c_bool, False)
        self.__frame_time = 0
        self.__frame_buff = None

        self.__threads_processes = ()

    def _captureProc(self):
        t0 = time.time()
        cap = cv2.VideoCapture(*self.__video_args)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_size[1])
        cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        while self.__live.value:
            try:
                frame_time = time.time() - t0
                t0 = time.time()
                flag, frame = cap.read()
                if flag:
                    self.__queue_frame.put((frame, frame_time))
            except:
                continue

    def __core(self):
        while self.__live.value:
            try:
                self.__frame_buff, self.__frame_time = self.__queue_frame.get(timeout=0.5)
            except:
                continue

    def read(self):
        return self.__frame_buff is None, self.__frame_buff

    def getFrame(self):
        return self.__frame_buff

    def getFPS(self):
        if not self.__frame_time:
            fr = 0
        else:
            fr = 1 / self.__frame_time
        return fr

    def start(self):
        if self.__live.value:
            return
        self.__live.value = True
        p = Process(target=self._captureProc)
        p.start()
        thr = threading.Thread(target=self.__core)
        thr.start()

        self.__threads_processes = (p, thr)

    def stop(self):
        if not self.__live.value:
            return
        self.__live.value = False
        for ele in self.__threads_processes:
            ele.join()
        self.__threads_processes = ()


class Scanner:

    def __init__(self, pipe):
        assert isinstance(pipe, CameraPipe)
        self.cap = pipe
        flag, frame = self.cap.read()
        self.frame = frame
        self.roi = None

    def readFrame(self):

        flag, frame = self.cap.read()
        self.frame = frame

    def readROI(self):
        if self.roi is None:
            return "Please run {0} "
        while self.frame is None:
            self.readFrame()
        flag, frame = self.cap.read()
        self.frame = frame[self.roi[1]:self.roi[3], (self.roi[0]):(self.roi[2])]
        # self.frame = frame[(self.roi[0]):(self.roi[2]), self.roi[1]:self.roi[3]]

    def scanTargetSurface(self, thresh=100, area_H=1000000000, area_L=5000):
        """
            para : thresh, area_H, srea_L
            return : a list contains 4 points
        """
        while self.frame is None:
            self.readFrame()

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)  # 灰度
        flag, bina = cv2.threshold(self.frame, thresh, 255, cv2.THRESH_BINARY)
        blurred = cv2.bilateralFilter(gray, 2, 200, 200)  # 双边滤波降噪
        edged = cv2.Canny(blurred, 25, 200)  # 边缘识别
        # edged = cv2.dilate(edged, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀连接边缘
        # cv2.imshow("mask",edged)
        # cv2.waitKey(1)
        contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓
        shapepoint = None
        if len(contours) > 0:
            # 按轮廓面积降序排列
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            for c in contours:
                # 近似轮廓
                if area_H > cv2.contourArea(c) > area_L:
                    cv2.drawContours(self.frame, c, -1, (255, 0, 0), 2)
                    peri = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.1 * peri, True)
                    # 凸四边形判定
                    if (len(approx) == 4) & (not cv2.isContourConvex(c)):
                        # cv2.drawContours(self.frame, c, -1, (0, 255, 0), 2)
                        # x,y,w,h = cv2.minAreaRect(c)
                        x, y, w, h = cv2.boundingRect(c)
                        count = 0
                        for i in range(50):
                            specimen_point = (x + (w // 50) * i, y + h // 2)
                            print(specimen_point[0], specimen_point[1])
                            if bina[specimen_point[0], specimen_point[1], 0] == 255:
                                count += 1
                        print("count = ", count)
                        if count < 30:  # 是否识别到靶面
                            continue

                        # cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        # rect = cv2.minAreaRect(c)
                        # box = cv2.boxPoints(rect)
                        # box = np.int0(box)
                        # cv2.drawContours(self.frame,[box],0,(0,0,255),2)
                        shapepoint = approx
                        self.roi = [x, y, x + w, y + h]
                        break
        return shapepoint

    def scanTags(self, area_H=300, area_L=50, shaperate_H=0.86, shaperate_L=0.65):
        """
            para : area_h, area_L, shaprate_H, shaperate_L
            func : muilty scan tags
        """

        while self.frame is None:
            self.readFrame()

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)  # 灰度
        blurred = cv2.bilateralFilter(gray, 2, 200, 200)  # 双边滤波降噪
        edged = cv2.Canny(blurred, 25, 200)  # 边缘识别
        # edged = cv2.dilate(edged, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀连接边缘
        contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓
        centers = []
        paperCnt = None
        if len(contours) > 0:
            # 按轮廓面积降序排列
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            for cnt in contours:
                cv2.drawContours(self.frame, cnt, -1, (0, 255, 0), 2)
                area = cv2.contourArea(cnt)
                if area_H > area > area_L:
                    pos, size, ang = cv2.fitEllipse(cnt)
                    # x, y, w, h = cv2.boundingRect(cnt)        # rect boud method_1
                    # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    # cv2.circle(img,(x+w//2,y+h//2),2,(0,0,255),3)
                    # print(area/(w*h))
                    # if (0.6 * w * h <= area) & (0.87 * w * h >= area):  # 判定是否为圆 method_1
                    if (shaperate_H * size[0] * size[1]) > area > (shaperate_L * size[0] * size[1]):
                        cv2.circle(self.frame, tuple([int(ele) for ele in pos]), 2, (0, 0, 255), 3)
                        centers.append(pos)
        return centers

    def scanLaser(self, area_L=1, area_H=40, thresh=210):
        """
            para : area_L, area_H, thresh
        """

        while self.frame is None:
            self.readFrame()

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # frame_blue, frame_green ,gray = cv2.split(self.frame)
        # 期望:由b,g创建掩膜,于r通道按位与,得到光点掩膜
        # 程序待完成
        flag, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)  # 阈值化处理
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)
        if len(contours) == 0:
            return [0, 0]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        cnt = contours[0]
        # cv2.drawContours(frame, contours, 0, (0,255,0), 3)
        # print(cv2.contourArea(cnt))
        x, y, w, h = cv2.boundingRect(cnt)
        # cv2.imshow("frame", self.frame)
        # cv2.waitKey(1)
        if area_L < w * h < area_H:
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            # cv2.circle(frame, (x + w // 2, y + h // 2), 2, (0, 255, 0), 3)
            center = [x + w // 2, y + h // 2]
            return center


def test_phase1():

    cam = CameraPipe((2,), (320, 240), 60)

    cam.start()

    try:
        while True:
            frame = cam.getFrame()
            if frame is None:
                continue

            cv2.imshow("frame", frame)
            print("FPS: {:.2f}".format(cam.getFPS()))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    cam.stop()


def test_phase2():
    cap = CameraPipe((2,), (320, 240), 60)
    cap.start()

    test1 = Scanner(cap)

    test1.readFrame()
    a = test1.scanTargetSurface()
    print(a)

    test1.readROI()
    b = test1.scanTags()
    print(b)

    try:
        while 1:
            test1.readROI()
            c = test1.scanLaser()
            print(c)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

    cap.stop()


if __name__ == "__main__":

    #test_phase2()

    test_phase1()


