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


class LaserPitchControl(PID):

    def __init__(self, ctl_clt, kp=1, ki=0, kd=0, core_freq=50):
        super(LaserPitchControl, self).__init__(kp=kp, ki=ki, kd=kd, core_freq=core_freq)
        assert isinstance(ctl_clt, PTControl)

        self.ctl_clt = ctl_clt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = 1 / core_freq

        self.last_out = 0

    def coreTask(self):
        self.ctl_clt.speed_pitch = self.out - self.last_out
        self.last_out = self.out

    def start(self):
        if not self.ctl_clt.live:
            self.ctl_clt.connect()
        super(LaserPitchControl, self).start()
