#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: control
# Created on: 2022/6/1


from i2cylib.engineering import PID

if __name__ == "__main__":
    from htsocket import PTControl
else:
    from .htsocket import PTControl


class LaserControl(PID):

    def __init__(self, ctl_clt, kp=1, ki=0, kd=0, core_freq=50):
        super(LaserControl, self).__init__()
        assert isinstance(ctl_clt, PTControl)

        self.ctl_clt = ctl_clt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = 1 / core_freq

    def __thread_calculator(self):


    def start(self):
        if not self.ctl_clt.live:
            self.ctl_clt.connect()
        super(LaserControl, self).start()