#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: ht_PT
# Created on: 2022/5/31

import serial as s


TTY = "/dev/ttyUSB0"
BAUD_RATE = 115200


class HTPT:

    def __init__(self):
        self.__sclt = s.Serial(TTY, BAUD_RATE)
