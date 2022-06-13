#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: key
# Created on: 2022/6/13

import os
import time

KEY1 = 33
KEY2 = 45
KEY3 = 36


def setUpPin(pin):
    if not os.path.exists("/sys/class/gpio/gpio{}".format(pin)):
        os.system("sudo echo {} > /sys/class/gpio/export".format(pin))
    os.system("sudo gpio -g mode {} in".format(pin))
    os.system("sudo gpio -g mode {} up".format(pin))
    os.system("sudo echo 1 > /sys/class/gpio/gpio{}/active_low".format(pin))


def gpioRead(pin):
    with open("/sys/class/gpio/gpio{}/value".format(pin), "r") as f:
        ret = f.read(1)
        f.close()
    return int(ret)


def waitKeyRelease(pin, timeout=0):
    t0 = time.time()
    while gpioRead(pin):
        time.sleep(0.001)
        if timeout and time.time() - t0 > timeout:
            return False
    return True
