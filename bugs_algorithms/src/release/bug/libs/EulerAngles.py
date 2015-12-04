#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

from math import pi

class EulerAngles:
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.radToDeg = 180.0 / pi
        self.degToRad = 1.0 / self.radToDeg