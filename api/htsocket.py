#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: I2cyLib
# Filename: htsocket
# Created on: 2022/5/31

from i2cylib.serial import HTSocket
from i2cylib.utils.logger.logger import *


COM = "/dev/ttyS4"
BAUD_RATE = 115200


CENTER = (1200, 5250)


def move(clt, pitch, yaw):
    assert isinstance(clt, HTSocket)
    payload = int(pitch).to_bytes(2, 'big', signed=False)
    payload += int(yaw).to_bytes(2, 'big', signed=False)
    clt.send(b"\xcc\x11", payload)


def test():
    clt = HTSocket(COM, BAUD_RATE)
    clt.connect()
    move(clt, *CENTER)


if __name__ == '__main__':
    test()
