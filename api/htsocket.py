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

CENTER = (1070, 5150)


class PTControl(HTSocket):

    def __init__(self, port, baud_rate, ctl_freq=50, center=(800, 5150), speed_dead_zone=(-20, 20),
                 pitch_range=(820, 5400), yaw_range=(3750, 6750),
                 yaw_90=5000, pitch_90=4600, pitch_center=5300, distance_cm=100):
        super(PTControl, self).__init__(port, baud_rate)
        self.__delay_perstep = 1 / ctl_freq
        self.dead_zone = speed_dead_zone
        self.pitch_range = pitch_range
        self.yaw_range = yaw_range

        self.pitch = 0
        self.yaw = 0
        self.speed_pitch = 0
        self.speed_yaw = 0
        self.speed_x = 0
        self.speed_y = 0

        self.flag_core_running = False
        self.live = False
        self.core_time = 0
        self.center = center
        self.yaw_step = yaw_90 / 90
        self.pitch_step = pitch_90 / 90
        self.pitch_center = pitch_center
        self.distance = distance_cm

        self.__ang_conv = 180 / np.pi

    def __core(self):
        self.flag_core_running = True

        while self.live:
            try:
                t0 = time.time()

                if self.speed_x or self.speed_y:
                    dx = self.speed_x * self.core_time
                    dy = self.speed_y * self.core_time
                    x, y = self.getDist()
                    # print(x, y, x + dx, y + dy)
                    # print(self.getAng())
                    self.moveToDist(x - dx, y + dy)
                    # print(self.pitch, self.yaw)

                else:
                    self.pitch += self.speed_pitch * self.core_time
                    self.yaw += self.speed_yaw * self.core_time

                if self.pitch < self.pitch_range[0]:
                    self.pitch = self.pitch_range[0]
                elif self.pitch > self.pitch_range[1]:
                    self.pitch = self.pitch_range[1]

                if self.yaw < self.yaw_range[0]:
                    self.yaw = self.yaw_range[0]
                elif self.yaw > self.yaw_range[1]:
                    self.yaw = self.yaw_range[1]

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

    def getAng(self):
        theta = (self.pitch_center - self.pitch) / self.pitch_step
        gamma = (self.center[1] - self.yaw) / self.yaw_step
        return theta, gamma

    def getDist(self):
        theta, gamma = self.getAng()
        theta /= self.__ang_conv
        gamma /= -self.__ang_conv
        x = self.distance * np.tan(gamma)
        y = self.distance / (np.cos(gamma) * np.tan(theta))
        return x, y

    def moveTo(self, pitch=None, yaw=None):
        if pitch is None:
            pitch = self.center[0]
        if yaw is None:
            yaw = self.center[1]
        self.pitch = pitch
        self.yaw = yaw

    def moveToAng(self, theta=None, gamma=None):
        if theta is None:
            pitch = self.center[0]
        else:
            pitch = self.pitch_center - theta * self.pitch_step
        if gamma is None:
            yaw = self.center[1]
        else:
            yaw = self.center[1] + gamma * self.yaw_step
        self.pitch = pitch
        self.yaw = yaw

    def moveToDist(self, x=0, y=0):
        theta = np.arccos(y / np.power(self.distance**2 + x**2 + y**2, 0.5)) * self.__ang_conv
        gamma = np.arctan(x / self.distance) * self.__ang_conv
        self.moveToAng(theta, gamma)

    def moveDist(self, speed_x=0, speed_y=0):
        self.speed_x = speed_x
        self.speed_y = speed_y

    def moveAng(self, speed_pitch=0, speed_yaw=0):
        if self.dead_zone[1] > self.speed_yaw > self.dead_zone[0]:
            speed_yaw = 0
        if self.dead_zone[1] > self.speed_pitch > self.dead_zone[0]:
            speed_pitch = 0
        self.speed_pitch = speed_pitch
        self.speed_yaw = speed_yaw

    def smoothMoveToDist(self, start, to, accuracy=1, delay=0.02):
        distance = np.power((start[0] - to[0])**2 + (start[0] - to[0])**2, 0.5)
        dots = distance * accuracy
        x = np.linspace(start[0], to[0], dots)
        y = np.linspace(start[1], to[1], dots)
        for i, val in enumerate(x):
            self.moveToDist(val, y[i])
            time.sleep(delay)


def move(clt, pitch, yaw):
    assert isinstance(clt, PTControl)
    clt.moveTo(pitch, yaw)


def smoothMove(clt, start, to, dots=100, delay=0.02):
    assert isinstance(clt, PTControl)
    x = np.linspace(start[0], to[0], dots)
    y = np.linspace(start[1], to[1], dots)
    for i, val in enumerate(x):
        clt.moveToDist(val, y[i])
        time.sleep(delay)


def test():
    DELAY = 0.01
    clt = PTControl(COM, BAUD_RATE)
    clt.connect()

    clt.moveToDist(30, 0)

    for i in range(2):
        smoothMove(clt, (-80, 100), (80, 100), delay=DELAY, dots=300)
        time.sleep(0.2)
        smoothMove(clt, (80, 100), (-80, 100), delay=DELAY, dots=300)
        time.sleep(0.2)

    for i in range(2):
        smoothMove(clt, (23, 10), (23, 60), delay=DELAY)
        time.sleep(0.2)
        smoothMove(clt, (23, 60), (-27, 60), delay=DELAY)
        time.sleep(0.2)
        smoothMove(clt, (-27, 60), (-27, 10), delay=DELAY)
        time.sleep(0.2)
        smoothMove(clt, (-27, 10), (23, 10), delay=DELAY)
        time.sleep(0.2)
        smoothMove(clt, (23, 10), (-27, 60), delay=DELAY)
        time.sleep(0.2)
        smoothMove(clt, (-27, 60), (-27, 10), delay=DELAY)
        time.sleep(0.2)
        smoothMove(clt, (-27, 10), (23, 60), delay=DELAY)
        time.sleep(0.2)
        smoothMove(clt, (23, 60), (23, 10), delay=DELAY)
        time.sleep(0.5)

    time.sleep(0.5)
    clt.moveTo()
    time.sleep(0.5)

    clt.close()


if __name__ == '__main__':
    test()
