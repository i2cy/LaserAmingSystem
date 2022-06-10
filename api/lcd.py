#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: lcd
# Created on: 2022/6/10

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

import time


if __name__ == '__main__':
    def oled_test():
        clt = i2c(port=3, address=0x3C)

