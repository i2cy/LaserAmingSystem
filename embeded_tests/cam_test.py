#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: cam_test.py
# Created on: 2022/6/8

from cam_config import CAM
import matplotlib.pyplot as plt
import cv2

cap = CAM

while True:
    try:
        flag, frame = cap.read()
        if flag:
            plt.cla()
            plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            plt.pause(0.01)
    except KeyboardInterrupt:
        break

plt.close(1)
