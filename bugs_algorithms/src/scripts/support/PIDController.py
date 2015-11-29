#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'


class PID:
    def __init__(self, p=0.0, i=0.0, d=0.0):
        self.p = p
        self.i = i
        self.d = d

class PIDSettings:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, maxI=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.maxI = maxI

class PIDController:
    def __init__(self, frequency):
        self.frequency = frequency
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.ePrev = 0.0
        self.sumErr = 0.0
        self.maxSumErr = 3.4028234664e+38
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def setCoefficients(self, Kp, Ki, Kd ):

        self.Kp = Kp
        self.Ki = Ki / self.frequency
        self.Kd = Kd * self.frequency

    def setMaxI(self, maxSumErr ):
        self.maxSumErr = maxSumErr

    def settings(self):

        s = PIDSettings()
        s.Kp = self.Kp
        s.Ki = self.Ki * self.frequency
        s.Kd = self.Kd / self.frequency
        s.maxI = self.maxSumErr

        return s

    def output(self, currentErr):

        self.sumErr += currentErr
        if self.sumErr > self.maxSumErr:
            self.sumErr = self.maxSumErr
        self.p = self.Kp * currentErr
        self.i = self.Ki * self.sumErr
        self.d = self.Kd * ( currentErr - self.ePrev )
        u = self.p + self.i + self.d
        self.ePrev = currentErr

        return u

    def currentPID(self):

        pid = PID()
        pid.p = self.p
        pid.i = self.i
        pid.d = self.d

        return pid

    def reset(self):

        self.ePrev = 0.0
        self.sumErr = 0.0
        self.p = self.i = self.d = 0.0
