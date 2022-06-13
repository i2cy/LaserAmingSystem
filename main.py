#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: main
# Created on: 2022/6/2

import os
import time
from api.control import *
from api.htsocket import PTControl
from api.scanner import Scanner
from api.key import *


KEY1 = 33
KEY2 = 45
KEY3 = 36

FONT_SIZE = 2

XP, XI, XD = (1.18, 0, 0.12)
YP, YI, YD = (1.18, 0, 0.10)

PID_OUT_LIMIT = [-70, 70]
PID_DEATH_AREA = 1.7
FILTER_1 = 0.4
FILTER_2 = 0.4

LASER_ARGS = (1, 60, 200)

ISO_CROP = 50
ISO_EXP = 120
ISO_KP = 0.07


def twoDotsTest(controller):
    assert isinstance(controller, Control)
    ctl = controller
    sc = ctl.scanner
    clt = ctl.pidX.ctl_clt



def menu(controller):
    assert isinstance(controller, Control)
    ctl = controller
    sc = ctl.scanner
    clt = ctl.pidX.ctl_clt

    menu_num = 0

    clt.printLine("[Menu]", 1)
    clt.printLine("1. Scan ROI", 2)

    # print(gpioRead(KEY1), gpioRead(KEY2), gpioRead(KEY3))

    # 选择菜单
    while not gpioRead(KEY2):
        # print(gpioRead(KEY1), gpioRead(KEY2), gpioRead(KEY3))
        if gpioRead(KEY3):
            waitKeyRelease(KEY3)
            menu_num += 1
            if menu_num == 0:
                clt.printLine("1. Scan ROI", 2)
            elif menu_num == 1:
                clt.printLine("2. Scan Tags", 2)
            elif menu_num == 2:
                clt.printLine("3. Auto ISO", 2)
            elif menu_num == 3:
                clt.printLine("4. 2 Tag Move", 2)
            elif menu_num == 4:
                clt.printLine("5. 3 Tag Move", 2)
            elif menu_num == 5:
                clt.printLine("6. Draw Circle", 2)
            else:
                menu_num = 0
                clt.printLine("1. Scan ROI", 2)
        time.sleep(0.001)

    waitKeyRelease(KEY2)

    if menu_num == 0:
        # 扫描标靶获得ROI
        clt.printLine("Scanning:", 1)
        clt.printLine("Target", 2)
        print("scanning target surface")
        while sc.roi is None:
            try:
                sc.readFrame()
                sc.scanTargetSurface()
            except KeyboardInterrupt:
                break
        if sc.roi is not None:
            clt.printLine("ROI: {}x{}".format(sc.roi[2] - sc.roi[0], sc.roi[3] - sc.roi[1]), 3)
        print("ROI:", sc.roi)


def main():
    print("initializing...")

    # 修改串口资源权限
    os.system("sudo chmod 777 /dev/ttyS4")

    # 初始化HT云台控制接口
    clt = PTControl("/dev/ttyS4", 115200, font_size=FONT_SIZE)
    clt.moveToAng(70, 0)
    ang = clt.getAng()
    clt.moveToAng(0, 0)
    center = (0, 0)

    # 启动HT云台控制
    clt.connect()

    # 显示初始化信息
    clt.clearLine()
    clt.printLine("[Init]", 1)

    # 初始化按键
    setUpPin(KEY1)
    setUpPin(KEY2)
    setUpPin(KEY3)

    # 初始化X方向PID控制器
    pidx = LaserYawControl(clt, XP, XI, XD)
    pidx.out_limit = PID_OUT_LIMIT
    pidx.death_area = PID_DEATH_AREA
    pidx.integ_limit = [-10, 10]
    clt.printLine("X-Axis PID", 2)

    # 初始化Y方向PID控制器
    pidy = LaserPitchControl(clt, YP, YI, YD)
    pidy.out_limit = PID_OUT_LIMIT
    pidy.death_area = PID_DEATH_AREA
    pidy.integ_limit = [-10, 10]
    clt.printLine("Y-Axis PID", 2)

    # 初始化摄像头采集管道
    cap = CameraPipe((0,), (320, 240), analogue_gain=16, exposure=1660)
    cap.start()
    sc = Scanner(cap)
    clt.printLine("Cam Pipe", 2)

    # 初始化激光坐标控制系统
    ctrl = Control(pidx, pidy, sc, x_filter=FILTER_1, y_filter=FILTER_2, laser_args=LASER_ARGS)
    clt.printLine("Aim Sys", 2)

    # 等待摄像头采集管道就绪
    print("waiting for camera pipe line to be ready...")
    clt.printLine("FPS Steady", 2)
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
    clt.printLine("RkISP RST", 2)
    os.system("gst-camera.sh >/dev/null 2>/dev/null")
    time.sleep(3)
    os.system("stop-gst-camera.sh >/dev/null 2>/dev/null")

    # 将相机ISO调至最低
    clt.printLine("Auto ISO", 2)
    cap.setCamArgs(analogue_gain=16)
    time.sleep(2)

    # 自动调节相机ISO
    print("auto tuning ISO...")
    sc.autoISO(exp=ISO_EXP, peek_thresh=1000, p=ISO_KP, smooth_window=7, verbose=False, top_crop=ISO_CROP)
    print("\ncurrent ISO: {}".format(sc.iso))
    clt.printLine("ISO: {:1f}".format(sc.iso), 3)

    while True:
        try:
            menu(ctrl)
        except (Exception, KeyboardInterrupt) as err:
            print("exited:", err)
            break

    # 安全退出
    ctrl.stop()
    pidx.pause()
    pidy.pause()
    clt.close()
    cap.stop()
    print("")
    print("test end")


if __name__ == '__main__':
    main()
