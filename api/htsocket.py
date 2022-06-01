#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: I2cyLib
# Filename: htsocket
# Created on: 2022/5/31

import time
import numpy as np
from i2cylib.serial import HTSocket
from i2cylib.utils.logger.logger import *


COM = "/dev/ttyS4"
BAUD_RATE = 115200


CENTER = (1200, 5250)

DELAY = 0.01


def move(clt, pitch, yaw):
    assert isinstance(clt, HTSocket)
    payload = int(pitch).to_bytes(2, 'big', signed=False)
    payload += int(yaw).to_bytes(2, 'big', signed=False)
    clt.send(b"\xcc\x11", payload)


def test():
    clt = HTSocket(COM, BAUD_RATE)
    clt.connect()
    move(clt, *CENTER)
    time.sleep(1)

    p0 = (1040, 5520)
    p1 = (1380, 4950)

    move(clt, *p0)
    time.sleep(1)
    move(clt, *p1)
    time.sleep(1)

    x = np.linspace(p0[0], p1[0], 50)
    y = np.linspace(p0[1], p1[1], 50)

    xc = np.linspace(0, 2 * np.pi, 300)
    yc = np.linspace(0, 2 * np.pi, 300)

    xc = np.sin(xc)
    yc = np.cos(xc)

    R = 400

    for i2 in range(0, 3):
        for i, xi in enumerate(x):
            move(clt, int(xi), int(y[i]))
            time.sleep(DELAY)
        time.sleep(0.5)
        move(clt, *p0)
        time.sleep(0.5)

    for i2 in range(0, 3):
        for i, xi in enumerate(xc):
            move(clt, CENTER[0] + int(xi*R), CENTER[1] + int(yc[i]*R))
            time.sleep(DELAY)
        time.sleep(0.5)
        move(clt, *CENTER)
        time.sleep(0.5)

    time.sleep(1)
    move(clt, *CENTER)


if __name__ == '__main__':
    test()
