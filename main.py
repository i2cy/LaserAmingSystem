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
KEY2 = 35
KEY3 = 36

FONT_SIZE = 2

XP, XI, XD = (0.87, 0, 0.12)
YP, YI, YD = (0.87, 0, 0.10)

PID_OUT_LIMIT = [-70, 70]
PID_DEATH_AREA = 1.7
FILTER_1 = 0.4
FILTER_2 = 0.4

LASER_ARGS = (1, 60, 210)

ISO_CROP = 50
ISO_EXP = 120
ISO_KP = 0.06

TARGET_CENTER = None
CAM_CENTER = None
TAGS = None
TIMEOUT = 10


def twoDotsTest(controller):
    assert isinstance(controller, Control)
    ctrl = controller
    sc = ctrl.scanner
    clt = ctrl.pidX.ctl_clt


def compareDist(p1, p2, thresh=6):
    distance = np.power((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2, 0.5)
    return distance < thresh


def menu(controller):
    global TARGET_CENTER, CAM_CENTER, TAGS
    
    assert isinstance(controller, Control)
    ctrl = controller
    sc = ctrl.scanner
    clt = ctrl.pidX.ctl_clt
    cap = sc.cap
    pidx = ctrl.pidX
    pidy = ctrl.pidY

    menu_num = 0

    # 将激光点移动到靶面中心
    time.sleep(1)
    xa, ya = sc.getTargetCenter()
    ctrl.move(xa, -ya, wait=False)

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
                clt.printLine("3. Reinit", 2)
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
        sc.roi = None
        clt.printLine("[Scan ROI]", 1)
        clt.printLine("Scanning", 2)
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
    
    elif menu_num == 1:
        # 扫描标记
        clt.moveToAng(0)
        clt.printLine("[Scan Tag]", 1)
        clt.printLine("Tags", 2)
        time.sleep(0.5)
        TAGS = []
        try:
            sc.readROI()
            t = sc.scanTags()
            TAGS = []
            tag_loc = []
            for ele in t:
                TAGS.append((ele[0], -ele[1]))
                loc = sc.cvtCdt(ele)
                tag_loc.append((loc[0], -loc[1]))
            print("TAGS scanned:\n{}".format(tag_loc))
            clt.printLine("Tag found: {}".format(len(tag_loc)), 3)
        except Exception as err:
            print("failed to read TAGS,", err)
    
    elif menu_num == 2:  # 重新初始化
        # 扫描标靶获得ROI
        sc.roi = None
        clt.printLine("[Scan ROI]", 1)
        clt.printLine("Scanning", 2)
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

        # 启动激光坐标控制系统
        clt.printLine("Aim CTL", 2)
        print("starting control and debuging")
        ctrl.start()

        # 启动XY方向PID控制器
        clt.moveToAng(75)
        TARGET_CENTER = clt.getDist()
        time.sleep(1)
        pidx.start()
        pidy.start()

        # 校准云台方向
        clt.printLine("[Cali]", 1)
        clt.printLine("PT Center", 2)
        try:
            ctrl.move(0, 0, timeout=TIMEOUT, err_max=0.8)
            clt.center = (clt.pitch, clt.yaw)
            print("\nnew yaw center recorded: {:.2f}".format(clt.center[1]))
        except (KeyboardInterrupt, Exception) as err:
            print("exited: {}".format(err))

        # 移动到靶面中心点
        clt.printLine("[Moving]", 1)
        clt.printLine("Cam Center", 2)
        print("\n> moving to center spot")
        try:
            xa, ya = sc.getTargetCenter()
            print("center point: ({:.2f}, {:.2f})".format(float(xa), float(ya)))
            ctrl.move(xa, -ya, timeout=TIMEOUT)
            TARGET_CENTER = clt.getDist()
            print("\ncenter dist location:", TARGET_CENTER)
        except (Exception, KeyboardInterrupt) as err:
            print("exited: {}".format(err))
            clt.moveToDist(*TARGET_CENTER)
        
    elif menu_num == 3:  # 两点测试
        # 扫描标记
        locations = []

        clt.moveToAng(10)
        clt.printLine("[Scan Tag]", 1)
        clt.printLine("Tags", 2)
        time.sleep(0.5)
        TAGS = []

        try:
            sc.readROI()
            t = sc.scanTags()
            TAGS = []
            tag_loc = []
            for ele in t:
                TAGS.append((ele[0], -ele[1]))
                loc = sc.cvtCdt(ele)
                tag_loc.append((loc[0], -loc[1]))
            print("TAGS scanned:\n{}".format(tag_loc))
            clt.printLine("Tag found: {}".format(len(tag_loc)), 3)
        except Exception as err:
            print("failed to read TAGS,", err)

        xa, ya = sc.getTargetCenter()
        clt.moveToDist(*TARGET_CENTER)
        ctrl.move(xa, -ya, wait=False)
        time.sleep(0.5)
        ctrl.pidX.start()
        ctrl.pidY.start()
        time.sleep(0.5)

        if len(TAGS) >= 1:
            print("\n> moving to spot ({:.1f}, {:.1f})".format(*TAGS[0]))
            ctrl.move(*TAGS[0], timeout=TIMEOUT, stable_time=1.2)
            locations.append(clt.getDist())

        if len(TAGS) > 1:
            print("\n> moving to spot ({:.1f}, {:.1f})".format(*TAGS[1]))
            ctrl.move(*TAGS[1], timeout=TIMEOUT)
            locations.append(clt.getDist())

        if len(TAGS) > 2:
            print("\n> moving to spot ({:.1f}, {:.1f})".format(*TAGS[2]))
            ctrl.move(*TAGS[2], timeout=TIMEOUT)
            locations.append(clt.getDist())

        clt.printLine("[Tag Location]", 1)
        for i, (x, y) in enumerate(locations):
            clt.printLine(">{} ({:.1f}, {:.1f})".format(i + 1, - x + TARGET_CENTER[0], y - TARGET_CENTER[1]), i + 3)

    elif menu_num == 4:  # 三点测试
        # 扫描标记
        locations = []

        clt.moveToAng(10)
        clt.printLine("[Scan Tag]", 1)
        clt.printLine("Tags", 2)
        time.sleep(0.5)
        TAGS = []
        tag_temp = []

        while len(TAGS) < 3:
            clt.printLine("Tag found: {}".format(len(TAGS)), 3)
            try:
                sc.readROI()
                t = sc.scanTags()
                for ele in t:
                    tag_temp.append((ele[0], -ele[1]))
                for ele in TAGS:
                    for new in tag_temp[:]:
                        if compareDist(ele, new):
                            tag_temp.remove(new)
                TAGS += tag_temp
                time.sleep(1)
            except Exception as err:
                print("failed to read TAGS,", err)

        xa, ya = sc.getTargetCenter()
        clt.moveToDist(*TARGET_CENTER)
        ctrl.move(xa, -ya, wait=False)
        time.sleep(0.5)
        ctrl.pidX.start()
        ctrl.pidY.start()
        time.sleep(0.5)

        if len(TAGS) >= 1:
            print("\n> moving to spot ({:.1f}, {:.1f})".format(*TAGS[0]))
            ctrl.move(*TAGS[0], timeout=TIMEOUT, stable_time=1.2)
            locations.append(clt.getDist())

        if len(TAGS) > 1:
            print("\n> moving to spot ({:.1f}, {:.1f})".format(*TAGS[1]))
            ctrl.move(*TAGS[1], timeout=TIMEOUT)
            locations.append(clt.getDist())

        if len(TAGS) > 2:
            print("\n> moving to spot ({:.1f}, {:.1f})".format(*TAGS[2]))
            ctrl.move(*TAGS[2], timeout=TIMEOUT)
            locations.append(clt.getDist())

        clt.printLine("[Tag Location]", 1)
        for i, (x, y) in enumerate(locations):
            clt.printLine(">{} ({:.1f}, {:.1f})".format(i + 1, - x + TARGET_CENTER[0], y - TARGET_CENTER[1]), i + 2)

    time.sleep(2)

def main():
    global TARGET_CENTER, CAM_CENTER
    
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
    t_start = time.time()

    # 显示初始化信息
    clt.clearLine()
    time.sleep(0.2)
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
    PTctrl = Control(pidx, pidy, sc, x_filter=FILTER_1, y_filter=FILTER_2, laser_args=LASER_ARGS)
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
    sc.autoISO(exp=ISO_EXP, peek_thresh=1000, p=ISO_KP, smooth_window=7, verbose=True, top_crop=ISO_CROP)
    print("\ncurrent ISO: {}".format(sc.iso))
    clt.printLine("ISO: {:1f}".format(sc.iso), 3)

    # 扫描标靶获得ROI
    sc.roi = None
    clt.printLine("[Scan ROI]", 1)
    clt.printLine("Scanning", 2)
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

    # 启动激光坐标控制系统
    clt.printLine("Aim CTL", 2)
    print("starting control and debuging")
    PTctrl.start()

    # 启动XY方向PID控制器
    clt.moveToAng(*ang)
    TARGET_CENTER = clt.getDist()
    time.sleep(1)
    pidx.start()
    pidy.start()

    # 校准云台方向
    clt.printLine("[Cali]", 1)
    clt.printLine("PT Center", 2)
    try:
        PTctrl.move(0, 0, timeout=TIMEOUT, err_max=0.8)
        clt.center = (clt.pitch, clt.yaw)
        print("\nnew yaw center recorded: {:.2f}".format(clt.center[1]))
    except (KeyboardInterrupt, Exception) as err:
        print("exited: {}".format(err))

    # 移动到靶面中心点
    clt.printLine("[Moving]", 1)
    clt.printLine("Targ. Center", 2)
    print("\n> moving to center spot")
    try:
        x, y = sc.getTargetCenter()
        print("center point: ({:.2f}, {:.2f})".format(float(x), float(y)))
        PTctrl.move(x, -y, timeout=TIMEOUT)
        clt.printLine("Wait for:", 1)
        TARGET_CENTER = clt.getDist()
        print("\ncenter dist location:", TARGET_CENTER)
    except (Exception, KeyboardInterrupt) as err:
        print("exited: {}".format(err))
        clt.moveToDist(*TARGET_CENTER)

    # 初始化完成
    clt.printLine("IT: {:.2f}s".format(time.time() - t_start), 4)

    while True:
        try:
            menu(PTctrl)
        except (Exception, KeyboardInterrupt) as err:
            print("exited:", err)
            break

    # 安全退出
    clt.clearLine()
    PTctrl.stop()
    pidx.pause()
    pidy.pause()
    clt.close()
    cap.stop()
    print("")
    print("test end")


if __name__ == '__main__':
    main()
