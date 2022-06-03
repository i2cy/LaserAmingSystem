#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: camera
# Created on: 2022/6/3

import threading
import time
import cv2
import ctypes
from multiprocessing import Process, Value, Queue


__cap = None
__queue_frame = Queue(128)
__live = Value(ctypes.c_bool, False)
__frame_time = 0
__frame_buff = None
__threads_processes = ()


def init(video_capture):
    assert isinstance(video_capture, cv2.VideoCapture)
    global __cap
    __cap = video_capture


def __captureProc(cap):
    global __live, __queue_frame
    t0 = time.time()
    while __live.value:
        try:
            frame_time = time.time() - t0
            t0 = time.time()
            flag, frame = cap.read()
            if flag:
                __queue_frame.put((frame, frame_time))
        except:
            continue


def __core():
    global __frame_buff, __frame_time
    while __live.value:
        try:
            __frame_buff, __frame_time = __queue_frame.get(timeout=0.5)
        except:
            continue


def getFrame():
    return __frame_buff


def getFPS():
    if not __frame_time:
        fr = 0
    else:
        fr = 1 / __frame_time
    return fr


def start():
    global __threads_processes, __cap
    if __live.value:
        return
    __live.value = True
    p = Process(target=__captureProc, args=(__cap,))
    p.start()
    thr = threading.Thread(target=__core)
    thr.start()

    __threads_processes = (p, thr)


def stop():
    global __threads_processes
    if not __live.value:
        return
    __live.value = False
    for ele in __threads_processes:
        ele.join()
    __threads_processes = ()
