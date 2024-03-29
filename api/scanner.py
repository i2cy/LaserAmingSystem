#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: BoranLi(https://github.com/Spc2r)
# Contributor: I2cy(i2cy@outlook.com)

import threading
import time
import matplotlib.pyplot as plt
import signal
import cv2
import ctypes
from multiprocessing import Process, Value, Queue
import numpy as np
import os
from math import degrees as dg


class CameraPipe:

    def __init__(self, video_capture_args, frame_size, frame_rate=None,
                 exposure=None, analogue_gain=None):
        self.__video_args = video_capture_args
        self.frame_size = frame_size
        self.frame_rate = frame_rate
        self.exposure = exposure
        self.analogue_gain = analogue_gain
        self.__queue_frame = Queue(128)
        self.__live = Value(ctypes.c_bool, False)
        self.__frame_time = 0
        self.__frame_buff = None

        self.__threads_processes = ()

    def __del__(self):
        if self.__live.value:
            self.stop()

    def _captureProc(self):
        t0 = time.time()
        cap = cv2.VideoCapture(*self.__video_args)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_size[1])
        if os.name != "nt":
            self.setCamArgs(self.exposure, self.analogue_gain)
        if self.frame_rate is not None:
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

    def setCamArgs(self, exposure=None, analogue_gain=None):
        if os.name != "nt":
            if exposure is not None:
                os.system("sudo v4l2-ctl -c exposure={} -d /dev/video{}".format(
                    exposure, self.__video_args[0]))
            if analogue_gain is not None:
                os.system("sudo v4l2-ctl -c analogue_gain={} -d /dev/video{}".format(
                    analogue_gain, self.__video_args[0]))
            print("ISO:", analogue_gain)

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
            if isinstance(ele, Process):
                os.kill(ele.pid, signal.SIGKILL)
            else:
                ele.join()
        self.__threads_processes = ()


class Scanner:

    def __init__(self, pipe):
        """
        advanced CV object specially designed for TI Cup

        :param pipe: CameraPipe object or VideoCapture object
        """
        self.target_cords_LU = None
        assert isinstance(pipe, CameraPipe) or isinstance(pipe, cv2.VideoCapture)
        self.cap = pipe
        flag, frame = self.cap.read()
        self.frame = frame
        self.roi = None
        self.target_cords = None
        self.iso = 40

    def readFrame(self):
        """
        read one frame from pipeline

        :return: np.ndarray
        """

        flag, frame = self.cap.read()
        self.frame = frame

        return self.frame

    def readROI(self, offset=0):
        """
        get clipped ROI area from buffer

        :return: np.ndarray
        """

        if self.roi is None:
            raise Exception("no ROI recorded yet, use scanTargetSurface method for detection")
        while self.frame is None:
            self.readFrame()
        flag, frame = self.cap.read()

        a = self.roi[0] - offset
        b = self.roi[1] - offset
        c = self.roi[2] + offset
        d = self.roi[3] + offset

        if a < 0:
            a = 0
        if b < 0:
            b = 0
        if c > self.cap.frame_size[1]:
            c = self.cap.frame_size[1]
        if d > self.cap.frame_size[0]:
            d = self.cap.frame_size[0]

        self.frame = frame[b:d, a:c]
        # self.frame = frame[(self.roi[0]):(self.roi[2]), self.roi[1]:self.roi[3]]
        return self.frame

    def autoISO(self, exp=130, dead_zone=15, p=0.1, window=11, skip=20,
                smooth_window=7, peek_thresh=900, verbose=True):
        """
            auto adjust camera ISO
            return: int     error
        """

        def findPeak(bins):
            peak_pos = 0
            k = 0
            for i in range(skip, len(bins) - window):
                if k > peek_thresh:
                    peak_pos = i - 1
                k = sum(bins[i: i + window])
            return peak_pos

        def setISO_add(add_num):
            self.cap.setCamArgs(analogue_gain=self.iso + add_num)
            self.iso = self.iso + add_num

        while True:
            self.readFrame()
            isoarea = self.frame[
                      self.cap.frame_size[0] // 4: 3 * self.cap.frame_size[0] // 4, 0: self.cap.frame_size[1]]
            gray = cv2.cvtColor(isoarea, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (smooth_window, smooth_window), 0)
            smoothed, bins = np.histogram(gray.ravel(), 256, [0, 256])

            plt.cla()

            plt.plot(smoothed)
            plt.pause(0.1)
            dif = exp - findPeak(smoothed)

            if verbose:
                print("\rexp: {:.2f}\terr: {:.2f}\tout: {:.2f}   ".format(exp, dif, dif * p), end="")

            if dead_zone > dif > -dead_zone:
                break
            else:
                add_num = dif * p
                setISO_add(add_num)

        plt.close(1)

        return dif

    def scanTargetSurface(self, thresh=15, area_H=20000, area_L=4000, validation_thresh=20):
        """
        Target surface detector

        :param validation_thresh:
        :param thresh: int (0~255, default:100), brightness threshold for verification
        :param area_H: int (>0, default:45000), maxim area for filter
        :param area_L: int (>0, default:4000), minim area for filter
        :return: List(4), a list of coordinates of each corner of the target
        """
        self.frame = None
        while self.frame is None:
            self.readFrame()

        self.roi = None
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)  # 灰度
        flag, bina = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)

        blurred = cv2.bilateralFilter(gray, 2, 200, 200)  # 双边滤波降噪

        plt.cla()
        plt.imshow(blurred)
        plt.pause(2)

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
                    approx = cv2.approxPolyDP(c, 0.08 * peri, True)
                    # 凸四边形判定
                    if (len(approx) == 4) & (not cv2.isContourConvex(c)):
                        # cv2.drawContours(self.frame, c, -1, (0, 255, 0), 2)
                        # x,y,w,h = cv2.minAreaRect(c)
                        x, y, w, h = cv2.boundingRect(c)
                        count = 0
                        for i in range(50):
                            specimen_point = (x + w // 2, y + (h // 50) * i)
                            # print(specimen_point[0], specimen_point[1])
                            if bina[specimen_point[0], specimen_point[1]] == 255:
                                count += 1
                        # print("count = ", count)
                        if count < validation_thresh:  # 是否识别到靶面
                            continue
                        # cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        # rect = cv2.minAreaRect(c)
                        # box = cv2.boxPoints(rect)
                        # box = np.int0(box)
                        # cv2.drawContours(self.frame,[box],0,(0,0,255),2)
                        shapepoint = approx
                        self.target_cords = np.squeeze(approx)
                        self.target_cords_LU = self.target_cords
                        arr_t = self.target_cords.copy()
                        arr_t2 = self.target_cords.copy()

                        assert isinstance(arr_t, np.ndarray)
                        arr_t -= int(arr_t.mean())

                        for i, (x1, y1) in enumerate(arr_t):
                            if x1 < 0 and y1 < 0:
                                ind = 0
                            elif x1 > 0 and y1 < 0:
                                ind = 1
                            elif x1 > 0 and y1 > 0:
                                ind = 2
                            else:
                                ind = 3
                            self.target_cords[ind] = (arr_t2[i][0] - self.cap.frame_size[0] / 2,
                                                      arr_t2[i][1] - self.cap.frame_size[1] / 2)

                        self.roi = [x, y, x + w, y + h]
                        break
        return shapepoint

    def scanTags(self, area_H=300, area_L=2, shaperate_H=0.96, shaperate_L=0.54):
        """
            para : area_h, area_L, shaprate_H, shaperate_L
            func : muilty scan tags
        """

        if self.roi is None:
            self.readFrame()
        else:
            self.readROI()

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)  # 灰度
        blurred = cv2.bilateralFilter(gray, 2, 200, 200)  # 双边滤波降噪
        edged = cv2.Canny(blurred, 25, 200)  # 边缘识别
        # edged = cv2.dilate(edged, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀连接边缘
        contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓
        centers = []
        fixed_centers = []
        paperCnt = None

        if len(contours) > 0:
            # 按轮廓面积降序排列
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            for cnt in contours:
                cv2.drawContours(self.frame, cnt, -1, (0, 255, 0), 2)
                area = cv2.contourArea(cnt)

                if area_H > area > area_L and len(cnt) >= 5:
                    pos, size, ang = cv2.fitEllipse(cnt)
                    # x, y, w, h = cv2.boundingRect(cnt)        # rect boud method_1
                    # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    # cv2.circle(img,(x+w//2,y+h//2),2,(0,0,255),3)
                    # print(area/(w*h))
                    # if (0.6 * w * h <= area) & (0.87 * w * h >= area):  # 判定是否为圆 method_1
                    print("ellipse: {:.4f}".format(area / (np.pi * size[0] * size[1] / 4)))

                    if (shaperate_H * np.pi * size[0] * size[1] / 4) > area > (
                            shaperate_L * np.pi * size[0] * size[1] / 4):
                        cv2.circle(self.frame, tuple([int(ele) for ele in pos]), 2, (0, 0, 255), 3)
                        if self.roi is not None:
                            pos_newsys = [pos[0] + self.roi[0] - self.cap.frame_size[0] / 2,
                                          pos[1] + self.roi[1] - self.cap.frame_size[1] / 2]
                            fixed_pos = self.cvtCdt(pos_newsys)
                            fixed_centers.append((fixed_pos[0] - 25, fixed_pos[1] - 25))
                        else:
                            pos_newsys = [pos[0] - self.cap.frame_size[0] / 2,
                                          pos[1] - self.cap.frame_size[1] / 2]
                        centers.append(pos_newsys)
        return centers, fixed_centers

    def scanLaser(self, area_L=1, area_H=40, thresh=210):
        """
        Laser beam coordinate detector

        :param area_L: int (>0, default:1)
        :param area_H: int (>0, default:40)
        :param thresh: int (0~255, default:210), threshold of brightness for filter
        :return:
        """

        if self.roi is None:
            self.readFrame()
        else:
            self.readROI()

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # frame_blue, frame_green ,gray = cv2.split(self.frame)
        # 期望:由b,g创建掩膜,于r通道按位与,得到光点掩膜
        # 程序待完成
        flag, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)  # 阈值化处理
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1)
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
            center_roi = [x + w / 2, y + h / 2]
            if self.roi is not None:
                center = [self.roi[0] + center_roi[0] - self.cap.frame_size[0] / 2,
                          self.roi[1] + center_roi[1] - self.cap.frame_size[1] / 2]
                center_roi = self.cvtCdt(center)
                center = (center_roi[0] - 25,
                          center_roi[1] - 25)
            else:
                center = [center_roi[0] - self.cap.frame_size[0] / 2,
                          center_roi[1] - self.cap.frame_size[1] / 2]
            return center

    def cvtCdt(self, dotpos):
        """
            func: Enter the roi coordinates, return the corrected target coordinates, and add them to self.act_laser_pos
        """
        if self.roi is not None:
            magnification = (self.target_cords[3][1] - self.target_cords[0][1]) / (
                        self.target_cords[3][0] - self.target_cords[0][0])
            magnification_2 = (self.target_cords[2][1] - self.target_cords[1][1]) / (
                        self.target_cords[2][0] - self.target_cords[1][0])
            relative_length = (self.target_cords[1][0] - self.target_cords[0][0]) - (
                        dotpos[1] - self.target_cords[0][1]) / magnification + (
                                          dotpos[1] - self.target_cords[0][1]) / magnification_2
            relative_pos_x = dotpos[0] - self.target_cords[0][0] + (dotpos[1] - self.target_cords[0][1]) / magnification
            actual_pos_x = 50 * relative_pos_x / relative_length
            # actual_pos_y = (dotpos[1] - self.target_cords[0][1])(self.target_cords[3][1] - self.target_cords[0][1])
            relative_pos_y = (dotpos[1] - self.target_cords[0][1]) * 0.5 * sum(
                [(self.target_cords[1][0] - self.target_cords[0][0]),
                 (self.target_cords[2][0] - self.target_cords[3][0])]) / (
                                         self.target_cords[3][1] - self.target_cords[0][1])
            actual_pos_y = 50 * relative_pos_y / (self.target_cords[3][1] - self.target_cords[0][1])
            coordinate_res = [actual_pos_x, actual_pos_y]
            # print(coordinate_res)
            return coordinate_res
        else:
            return dotpos

    def cvtCdt_ROI2Center(self, actPos):

        magnification = (self.target_cords_LU[3][1] - self.target_cords_LU[0][1]) / \
                        (self.target_cords_LU[3][0] - self.target_cords_LU[0][0])
        px_y = ((self.target_cords_LU[2][0] - self.target_cords_LU[3][0]) / 50) * \
               (self.target_cords_LU[3][1] - actPos[1]) / magnification
        px_x = (px_y / magnification) + \
               (self.target_cords_LU[2][0] - self.target_cords_LU[3][0] - 2 * px_y / magnification) * actPos[0] / 50
        roi_x = int(px_x) + self.target_cords_LU[3][0]
        roi_y = self.target_cords_LU[3][1] - int(px_y)
        center_x = roi_x - self.cap.frame_size[0] / 2
        center_y = roi_y - self.cap.frame_size[1] / 2
        centerPos = [center_x, center_y]
        return (centerPos)

    def pnpSolve(self):
        if self.roi is None:
            self.scanTargetSurface()

        f = 3.4
        dx = 0.01
        dy = 0.01
        u0 = 320
        v0 = 240
        list1 = [f / dx, 0, u0, 0, f / dy, v0, 0, 0, 1]
        mtx = np.mat(list1).reshape(3, 3)
        dist = np.mat([0, 0, 0, 0, 0])
        # dist = None

        matrix_img = np.mat(self.target_cords, dtype=np.float32)
        # objp=np.zeros((10*10,3),np.float32)
        # objp[:, :2]=np.mgrid[0:200:20, 0:200:20].T.reshape(-1,2)
        matrix_obj = np.mat([[0, 0, 0], [50, 0, 0], [50, 50, 0], [0, 50, 0]], dtype=np.float32)
        _, R, T = cv2.solvePnP(matrix_obj, matrix_img, mtx, dist)
        # _,R,T=cv2.solvePnP(matrix_obj,self.matrix_img,mtx,dist,flags=cv2.SOLVEPNP_P3P)
        sita_x = dg(R[0][0])
        sita_y = dg(R[1][0])
        sita_z = dg(R[2][0])
        print("sita_x is  ", sita_x)
        print("y轴旋转角 is  ", sita_y)
        print("sita_z is  ", sita_z)

        return sita_y


if __name__ == "__main__":
    def test_phase1():
        cam = CameraPipe((0,), (320, 240))

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
        cap = CameraPipe((0,), (320, 240))
        cap.start()

        test1 = Scanner(cap)

        test1.readFrame()
        a = test1.scanTargetSurface()
        print(test1.target_cords.shape)

        test1.pnpSolve()

        test1.readROI()
        b = test1.scanTags()
        print(b)

        try:
            while 1:
                test1.readROI()
                c = test1.scanLaser()
                print(c, " ", end="\r")
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

        cap.stop()


    def test_PnPslv():
        cap = CameraPipe((0,), (320, 240))
        cap.start()
        test0 = Scanner(cap)
        # test0.target_cords = np.array([[58, 45], [59, 124], [136, 125], [136, 45]], dtype=np.float32)
        while True:
            try:
                test0.scanTargetSurface()
                if test0.roi is None:
                    continue
                test0.readROI()
                test0.pnpSolve()

                print(test0.target_cords)
                if test0.frame.shape[0] > 0 and test0.frame.shape[1] > 0:
                    cv2.imshow("test", cv2.resize(test0.frame.copy(), (100, 100)))
                    cv2.waitKey(1)
                time.sleep(1)
            except KeyboardInterrupt:
                cap.stop()


    # test_PnPslv()

    test_phase2()

    # test_phase1()
