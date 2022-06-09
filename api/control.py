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


P, I, D = (1.24, 0.01, 0.14)  # initial tuned values


class Control:

    def __init__(self, x_PID, y_PID, scanner, laser_args=(), core_freq=50,
                 x_filter=0.1, y_filter=0.1, verbose=True):
        assert isinstance(x_PID, LaserYawControl)
        assert isinstance(y_PID, LaserPitchControl)
        assert isinstance(scanner, Scanner)
        self.pidX = x_PID
        self.pidY = y_PID

        self.scanner = scanner
        self.x_filter = x_filter
        self.y_filter = y_filter
        self.laser_args = laser_args
        self.verbose = verbose

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
            try:
                self.scanner.readROI(offset=10)
            except:
                self.scanner.readFrame()
            coord = self.scanner.scanLaser(*self.laser_args)

            if coord is None:
                continue

            coord = self.scanner.cvtCdt(coord)

            if self.verbose:
                print("\rLaser Beam at ({}, {})   ".format(*coord), end="")

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

    def isStable(self, err_max=2):
        return ctrl.pidX.err < err_max and ctrl.pidY.err < err_max

    def move(self, x, y, wait=True, timeout=0.0, err_max=2, stable_time=0.5):
        self.pidX.expectation = x
        self.pidY.expectation = y
        if wait:
            cnt = 0
            while self.isStable(err_max) and cnt < stable_time * 60:
                cnt += 1
                time.sleep(0.05)
            t0 = time.time()
            cnt = 0
            while cnt < stable_time * 20:
                if timeout:
                    if time.time() - t0 > timeout:
                        raise Exception("timeout")
                if self.isStable(err_max):
                    cnt += 1
                time.sleep(0.05)

    def targetCenterofCam(self):
        self.pidX.expectation = self.scanner.cap.frame_size[0] / 2
        self.pidY.expectation = self.scanner.cap.frame_size[1] / 2

    def init(self):

        # 初始化CV扫描器
        while self.scanner.roi is None:
            self.scanner.scanTargetSurface()
            pass


class LaserYawControl(PID):

    def __init__(self, ctl_clt, kp=1.0, ki=0.0, kd=0.0, core_freq=50):
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

    def __init__(self, ctl_clt, kp=1.0, ki=0.0, kd=0.0, core_freq=50):
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

    TEST_TIME = 6

    print("initializing")

    # 修改串口资源权限
    os.system("sudo chmod 777 /dev/ttyS4")

    # 初始化HT云台控制接口
    clt = PTControl("/dev/ttyS4", 115200)
    clt.moveToAng(80, 0)
    ang = clt.getAng()
    clt.moveToAng(0, 0)

    # 启动HT云台控制
    clt.connect()

    # 初始化X方向PID控制器
    pidx = LaserYawControl(clt, P, I, D)
    pidx.out_limit = [-40, 40]
    pidx.death_area = 2.5
    pidx.integ_limit = [-10, 10]

    # 初始化Y方向PID控制器
    pidy = LaserPitchControl(clt, P, I, D)
    pidy.out_limit = [-40, 40]
    pidy.death_area = 2.5
    pidy.integ_limit = [-10, 10]

    # 初始化摄像头采集管道
    cap = CameraPipe((0,), (320, 240), analogue_gain=16, exposure=1660)
    cap.start()
    sc = Scanner(cap)

    # 初始化激光坐标控制系统
    ctrl = Control(pidx, pidy, sc, x_filter=0.4, y_filter=0.4, laser_args=(1, 60, 220))

    # 等待摄像头采集管道就绪
    print("waiting for camera pipe line to be ready...")
    fps = cap.getFPS()
    cnt = 0
    while fps < 10 and cnt < 6:
        fps = cap.getFPS()
        print("\rcamera pipeline core frame rate: {:.2f}  ".format(fps), end="")
        if fps >= 10:
            cnt += 1
        time.sleep(0.5)
    print("")

    # 使用rkisp让相机自动曝光一次
    print("reinitializing camera arguments with gst-launcher")
    os.system("gst-camera.sh >nul 2>nul")
    time.sleep(3)
    os.system("stop-gst-camera.sh >nul 2>nul")

    # 将相机ISO调至最低
    cap.setCamArgs(analogue_gain=16)
    time.sleep(2)

    # 自动调节相机ISO
    print("auto tuning ISO...")
    sc.autoISO(exp=130, peek_thresh=1000, p=0.15, smooth_window=5)
    print("\ncurrent ISO: {}".format(sc.iso))
    time.sleep(1)

    # 预览图象
    while sc.frame is None:
        sc.readFrame()
    plt.cla()
    plt.imshow(sc.frame)
    plt.pause(2)
    plt.close(1)

    # 扫描标靶获得ROI
    print("scanning target surface")
    while sc.roi is None:
        try:
            sc.readFrame()
            sc.scanTargetSurface()
        except KeyboardInterrupt:
            break
    print("ROI:", sc.roi)

    # 预览ROI
    time.sleep(0.5)
    if sc.roi is not None:
        sc.readROI(offset=10)
        plt.cla()
        plt.imshow(sc.frame)
        plt.pause(2)
        plt.close(1)

    # 扫描标记
    tags = []
    try:
        sc.readROI()
        t = sc.scanTags()
        tags = []
        for ele in t:
            tags.append((ele[0], -ele[1]))
        print("tags scanned:\n{}".format(tags))
    except Exception as err:
        print("failed to read tags,", err)

    # 启动激光坐标控制系统
    print("starting control and debuging")
    ctrl.start()

    # 启动XY方向PID控制器
    clt.moveToAng(*ang)
    time.sleep(2)
    pidx.start()
    pidy.start()

    # 移动到视野中心点
    print("\n> moving to center spot")
    try:
        ctrl.move(*sc.cvtCdt((0, 0)), timeout=4)
    except Exception as err:
        clt.moveToAng(80, 0)

    # 依次移动到标靶4角以内的区域
    if sc.roi is not None:
        try:
            for x, y in sc.target_cords:
                ctrl.move(*sc.cvtCdt((x, y)), timeout=4)
            ctrl.move(*sc.cvtCdt((0, 0)), timeout=4)
        except Exception as err:
            print("failed")
            clt.moveToAng(80, 0)

    # 启用日志记录
    print("recording")
    ctrl.startDebug()

    # 开始测试
    t0 = time.time()
    try:
        for ele in range(4):
            if len(tags) >= 1:
                print("\n> moving to spot ({:.1f}, {:.1f})".format(*sc.cvtCdt(tags[0])))
                ctrl.move(*sc.cvtCdt(tags[0]))
            else:
                print("\n> moving to spot {}".format(sc.cvtCdt((10, 10))))
                ctrl.move(*sc.cvtCdt((10, 10)))

            if len(tags) > 1:
                print("\n> moving to spot ({:.1f}, {:.1f})".format(*sc.cvtCdt(tags[1])))
                ctrl.move(*sc.cvtCdt(tags[1]))
            else:
                print("\n> moving to spot {}".format(sc.cvtCdt((-10, -10))))
                ctrl.move(*sc.cvtCdt((-10, -10)))

            if len(tags) > 2:
                print("\n> moving to spot ({:.1f}, {:.1f})".format(*sc.cvtCdt(tags[2])))
                ctrl.move(*sc.cvtCdt(tags[2]))
            else:
                continue

    except KeyboardInterrupt:
        pass

    # 结束测试（安全退出）
    ctrl.stopDebug()
    ctrl.stop()
    pidx.pause()
    pidy.pause()
    clt.close()
    cap.stop()
    print("")
    print("test end")

    # 绘制X方向PID采样数据
    x = np.linspace(0, time.time() - t0, len(ctrl.x_out))
    plt.subplot(211)
    plt.title("X")
    plt.plot(x, ctrl.x_exp, color="g")
    plt.plot(x, ctrl.x_mea, color="b")
    plt.plot(x, ctrl.x_out, color="r")
    plt.legend("EMO")

    # 绘制Y方向PID采样数据
    plt.subplot(212)
    plt.title("Y")
    plt.plot(x, ctrl.y_exp, color="g")
    plt.plot(x, ctrl.y_mea, color="b")
    plt.plot(x, ctrl.y_out, color="r")
    plt.legend("EMO")
    plt.show()
