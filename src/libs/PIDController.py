#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'


class PIDController:
    def __init__(self, frequency):
        self.frequency = frequency
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.ePrev = 0.0
        self.sum_err = 0.0
        self.max_sum_err = 3.4028234664e+38
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def set_coefficients(self, Kp, Ki, Kd ):
        self.Kp = Kp
        self.Ki = Ki / self.frequency
        self.Kd = Kd * self.frequency

    def output(self, current_err):
        self.sum_err += current_err
        if self.sum_err > self.max_sum_err:
            self.sum_err = self.max_sum_err
        self.p = self.Kp * current_err
        self.i = self.Ki * self.sum_err
        self.d = self.Kd * (current_err - self.ePrev)
        u = self.p + self.i + self.d
        self.ePrev = current_err
        return u
