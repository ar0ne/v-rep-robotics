#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import math

def angle_between_vectors(a, b):  # a -> b

    a = a.unitVector()
    b = b.unitVector()
    angle = math.acos(b.dot(a))
    if (a.multiply(b)).z > 0.0:
        return -angle
    return angle