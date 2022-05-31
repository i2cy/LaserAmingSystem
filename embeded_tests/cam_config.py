#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: cam_config
# Created on: 2022/5/31

import cv2

CAM = cv2.VideoCapture(
    'rkisp device=/dev/video1 io-mode=4 ! video/x-raw,format=NV12,width=600,height=450,framerate=120/60 ! videoconvert '
    '! appsink',
    cv2.CAP_GSTREAMER)
