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

def distance_between_points(pos_1, pos_2):
    return math.sqrt((pos_1.x - pos_2.x) ** 2 + (pos_1.y - pos_2.y) ** 2)