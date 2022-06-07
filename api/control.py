#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: control
# Created on: 2022/6/1

import os
import threading
import time

import cv2
import numpy as np
from i2cylib.engineering import PID

if __name__ == "__main__":
    from htsocket import PTControl
    from scanner import Scanner, CameraPipe
else:
    from .htsocket import PTControl
    from .scanner import Scanner, CameraPipe


P, I, D = (2.36, 4.75, 0.3)  # initial tuned values


class Control:

    def __init__(self, x_PID, y_PID, scanner, laser_args=(), core_freq=50,
                 x_filter=0.1, y_filter=0.1):
        assert isinstance(x_PID, LaserYawControl)
        assert isinstance(y_PID, LaserPitchControl)
        assert isinstance(scanner, Scanner)
        self.pidX = x_PID
        self.pidY = y_PID

        self.scanner = scanner
        self.x_filter = x_filter
        self.y_filter = y_filter
        self.laser_args = laser_args
        self.live = False
        self.core_time = 1 / core_freq
        self.__current_core_time = 0
        self.threads = []

        self.debug = False
        self.x_exp = []
        self.x_mea = []
        self.x_out = []
        self.y_exp = []
        self.y_mea = []
        self.y_out = []

    def coreDebug(self):
        if self.__current_core_time:
            ct = 1 / self.__current_core_time
        else:
            ct = 0
        return {"core_freq": ct}

    def startDebug(self):
        self.x_exp = []
        self.x_mea = []
        self.x_out = []
        self.y_exp = []
        self.y_mea = []
        self.y_out = []
        self.debug = True

    def stopDebug(self):
        self.debug = False

    def stop(self):
        self.live = False
        for ele in self.threads:
            ele.join()

    def start(self):
        if self.live:
            return
        self.live = True
        thr = threading.Thread(target=self.__core)
        thr.start()
        self.threads.append(thr)

    def __core(self):
        x_lpf = 0
        y_lpf = 0
        x_dlpf = 0
        y_dlpf = 0
        while self.live:
            t0 = time.time()
            self.scanner.readFrame()
            coord = self.scanner.scanLaser(*self.laser_args)

            if coord is None:
                continue

            print(coord)

            x_mea, y_mea = coord

            if not x_mea:
                x_mea = (x_lpf - x_dlpf) + x_lpf

            if not y_mea:
                x_mea = (x_lpf - x_dlpf) + x_lpf

            y_mea = -y_mea

            x_lpf += self.x_filter * (x_mea - x_lpf)
            x_dlpf += self.x_filter * (x_lpf - x_dlpf)

            y_lpf += self.y_filter * (y_mea - y_lpf)
            y_dlpf += self.y_filter * (y_lpf - y_dlpf)

            self.pidX.measures = x_dlpf
            self.pidY.measures = y_dlpf

            if self.debug:
                self.x_mea.append(x_dlpf)
                self.x_exp.append(self.pidX.expectation)
                self.x_out.append(self.pidX.out)

                self.y_mea.append(y_dlpf)
                self.y_exp.append(self.pidY.expectation)
                self.y_out.append(self.pidY.out)

            delay = self.core_time - t0 + time.time()
            if delay > 0:
                time.sleep(delay)
            self.__current_core_time = time.time() - t0

    def move(self, x, y):
        self.pidX.expectation = x
        self.pidY.expectation = y

    def targetCenterofCam(self):
        self.pidX.expectation = self.scanner.cap.frame_size[0] / 2
        self.pidY.expectation = self.scanner.cap.frame_size[1] / 2

    def init(self):

        # 初始化CV扫描器
        while self.scanner.roi is None:
            self.scanner.scanTargetSurface()
            pass


class LaserYawControl(PID):

    def __init__(self, ctl_clt, kp=1, ki=0, kd=0, core_freq=50):
        super(LaserYawControl, self).__init__(kp=kp, ki=ki, kd=kd, core_freq=core_freq)
        assert isinstance(ctl_clt, PTControl)

        self.ctl_clt = ctl_clt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = 1 / core_freq

    def coreTask(self):
        self.ctl_clt.speed_x = self.out

    def start(self):
        if not self.ctl_clt.live:
            self.ctl_clt.connect()
        super(LaserYawControl, self).start()


class LaserPitchControl(PID):

    def __init__(self, ctl_clt, kp=1, ki=0, kd=0, core_freq=50):
        super(LaserPitchControl, self).__init__(kp=kp, ki=ki, kd=kd, core_freq=core_freq)
        assert isinstance(ctl_clt, PTControl)

        self.ctl_clt = ctl_clt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = 1 / core_freq

    def coreTask(self):
        self.ctl_clt.speed_y = self.out

    def start(self):
        if not self.ctl_clt.live:
            self.ctl_clt.connect()
        super(LaserPitchControl, self).start()


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    TEST_TIME = 10

    print("initializing")
    os.system("sudo chmod 777 /dev/ttyS4")
    clt = PTControl("/dev/ttyS4", 115200)
    clt.connect()
    clt.moveTo()

    pidx = LaserYawControl(clt, P, I, D)
    pidx.out_limit = [-100, 100]
    pidy = LaserPitchControl(clt, P, I, D)
    pidy.out_limit = [-100, 100]

    cap = CameraPipe((0,), (320, 240), analogue_gain=60)
    cap.start()
    sc = Scanner(cap)

    ctrl = Control(pidx, pidy, sc, x_filter=0.6, y_filter=0.6, laser_args=(1, 60, 210))
    ctrl.move(-15, -15)
    time.sleep(5)
    sc.readFrame()
    sc.scanTargetSurface()
    plt.imshow(cv2.cvtColor(sc.frame, cv2.COLOR_BGR2RGB))
    plt.show()
    print("starting control and debuging")
    ctrl.start()
    time.sleep(3)
    print("recording")
    ctrl.startDebug()
    pidx.start()
    pidy.start()
    time.sleep(TEST_TIME / 2)
    ctrl.move(10, 10)
    time.sleep(TEST_TIME / 2)
    ctrl.stopDebug()

    ctrl.stop()
    pidx.pause()
    pidy.pause()
    clt.close()
    cap.stop()

    x = np.linspace(0, TEST_TIME, len(ctrl.x_out))
    plt.subplot(211)
    plt.plot(x, ctrl.x_exp, color="g")
    plt.plot(x, ctrl.x_mea, color="b")
    plt.plot(x, ctrl.x_out, color="r")
    plt.legend("EMO")

    plt.subplot(212)
    plt.plot(x, ctrl.y_exp, color="g")
    plt.plot(x, ctrl.y_mea, color="b")
    plt.plot(x, ctrl.y_out, color="r")
    plt.legend("EMO")

    plt.show()
