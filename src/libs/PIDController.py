#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'


class PIDController:
    def __init__(self, frequency):
        self.frequency = frequency
        self.k_p = 0.0
        self.k_i = 0.0
        self.k_d = 0.0
        self.e_prev = 0.0
        self.sum_err = 0.0
        self.max_sum_err = 3.4028234664e+38
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def set_coefficients(self, k_p, k_i, k_d ):
        self.k_p = k_p
        self.k_i = k_i / self.frequency
        self.k_d = k_d * self.frequency

    def output(self, current_err):
        self.sum_err += current_err
        if self.sum_err > self.max_sum_err:
            self.sum_err = self.max_sum_err
        self.p = self.k_p * current_err
        self.i = self.k_i * self.sum_err
        self.d = self.k_d * (current_err - self.e_prev)
        u = self.p + self.i + self.d
        self.e_prev = current_err
        return u
