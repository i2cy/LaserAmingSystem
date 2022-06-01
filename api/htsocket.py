#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: I2cyLib
# Filename: htsocket
# Created on: 2022/5/31
import threading
import time
import numpy as np
from i2cylib.serial import HTSocket
from i2cylib.utils.logger.logger import *


COM = "/dev/ttyS4"
BAUD_RATE = 115200


CENTER = (1200, 5250)


class PTControl(HTSocket):

    def __init__(self, port, baud_rate, ctl_freq=50):
        super(PTControl, self).__init__(port, baud_rate)
        self.pitch = 0
        self.yaw = 0
        self.speed_pitch = 0
        self.speed_yaw = 0
        self.__delay_perstep = 1 / ctl_freq
        self.flag_core_running = False
        self.live = False
        self.core_time = 0

    def __core(self):
        self.flag_core_running = True
        while self.live:
            try:
                t0 = time.time()

                if self.speed_pitch:


                payload = int(self.pitch).to_bytes(2, 'big', signed=False)
                payload += int(self.yaw).to_bytes(2, 'big', signed=False)
                self.send(b"\xcc\x11", payload)
                delay = self.__delay_perstep - time.time() + t0
                if delay > 0:
                    time.sleep(delay)
                self.core_time = time.time() - t0
            except:
                continue

        self.flag_core_running = False

    def connect(self, port=None, baudrate=None, timeout=None):
        super(PTControl, self).connect(port=port, baudrate=baudrate, timeout=timeout)
        self.live = True
        if not self.flag_core_running:
            threading.Thread(target=self.__core).start()

    def close(self):
        self.live = False
        while self.flag_core_running:
            time.sleep(0.001)
        super(PTControl, self).close()

    def debug(self):
        return {"core_freq": 1 / self.core_time}

    def move_to(self, pitch, yaw):
        self.pitch = pitch
        self.yaw = yaw

    def move(self, speed_pitch, speed_yaw):




def move(clt, pitch, yaw):
    assert isinstance(clt, HTSocket)



def test():
    DELAY = 0.01
    clt = HTSocket(COM, BAUD_RATE)
    clt.connect()
    move(clt, *CENTER)
    time.sleep(1)

    p0 = (1040, 5520)
    p1 = (1360, 4950)

    move(clt, *p0)
    time.sleep(1)
    move(clt, *p1)
    time.sleep(1)

    x = np.linspace(p0[0], p1[0], 50)
    y = np.linspace(p0[1], p1[1], 50)

    xc = np.linspace(0, 2 * np.pi, 300)
    yc = np.linspace(0, 2 * np.pi, 300)

    xc = np.sin(xc)
    yc = np.cos(yc)

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
