#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: multi_proc
# Created on: 2022/6/3

import cv2
from multiprocessing import Queue, Process, Pool


def camera_loop(cam):

    assert isinstance(cam, cv2.VideoCapture)