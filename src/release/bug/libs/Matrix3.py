#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import math
from Vector3 import Vector3
from Quaternion import Quaternion


class Matrix3:
    def __init__(self, m=None):
        if m is None:
            self.m = [[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
        else:
            self.m = m

    def determinant(self):
        return self.m[0][0] * self.m[1][1] * self.m[2][2] - self.m[0][0] * self.m[1][2] * self.m[2][1] - self.m[0][1] * self.m[1][0] * self.m[2][2] + self.m[0][1] * self.m[1][2] * self.m[2][0] + self.m[0][2] * self.m[1][0] * self.m[2][1] - self.m[0][2] * self.m[1][1] * self.m[2][0]

    def inverse(self):
        inv = Matrix3()
        det = self.m[0][0] * self.m[1][1] * self.m[2][2] - self.m[0][0] * self.m[1][2] * self.m[2][1] - self.m[0][1] * self.m[1][0] * self.m[2][2] + self.m[0][1] * self.m[1][2] * self.m[2][0] + self.m[0][2] * self.m[1][0] * self.m[2][1] - self.m[0][2] * self.m[1][1] * self.m[2][0]
        invDet = 1.0 / det

        inv.m[0][0] = ( self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1] ) * invDet
        inv.m[0][1] = ( self.m[0][2] * self.m[2][1] - self.m[0][1] * self.m[2][2] ) * invDet
        inv.m[0][2] = ( self.m[0][1] * self.m[1][2] - self.m[0][2] * self.m[1][1] ) * invDet

        inv.m[1][0] = ( self.m[2][0] * self.m[1][2] - self.m[1][0] * self.m[2][2] ) * invDet
        inv.m[1][1] = ( self.m[0][0] * self.m[2][2] - self.m[0][2] * self.m[2][0] ) * invDet
        inv.m[1][2] = ( self.m[0][2] * self.m[1][0] - self.m[0][0] * self.m[1][2] ) * invDet

        inv.m[2][0] = ( self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0] ) * invDet
        inv.m[2][1] = ( self.m[0][1] * self.m[2][0] - self.m[0][0] * self.m[2][1] ) * invDet
        inv.m[2][2] = ( self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0] ) * invDet

        return inv

    def multiply(self, v):
        res = Vector3()
        res.x = self.m[0][0] * v[0] + self.m[0][1] * v[1] + self.m[0][2] * v[2]
        res.y = self.m[1][0] * v[0] + self.m[1][1] * v[1] + self.m[1][2] * v[2]
        res.z = self.m[2][0] * v[0] + self.m[2][1] * v[1] + self.m[2][2] * v[2]

        return res

    def toQuaternion(self):

        q = Quaternion()

        t = self.m[0][0] + self.m[1][1] + self.m[2][2] + 1

        if t > 0:
            s = 0.5 / math.sqrt(t)
            q.w = 0.25 / s
            q.x = (self.m[2][1] - self.m[1][2]) * s
            q.y = (self.m[0][2] - self.m[2][0]) * s
            q.z = (self.m[1][0] - self.m[0][1]) * s

            return q

        max = self.m[0][0]

        iColMax = 0

        for iRow in range(0, 3):
            for iCol in range(0, 3):
                if self.m[iRow][iCol] > max:
                    max = self.m[iRow][iCol]
                    iColMax = iCol

        if iColMax == 0:
            s = 1.0 / (math.sqrt(1.0 + self.m[0][0] - self.m[1][1] - self.m[2][2]) * 2.0)
            q.w = ( self.m[1][2] + self.m[2][1] ) * s
            q.x = 0.5 * s
            q.y = ( self.m[0][1] + self.m[1][0] ) * s
            q.z = ( self.m[0][2] + self.m[2][0] ) * s
        elif iColMax == 1:
            s = 1.0 / ( math.sqrt( 1.0 + self.m[1][1] - self.m[0][0] - self.m[2][2] ) * 2.0 )
            q.w = ( self.m[0][2] + self.m[2][0] ) * s
            q.x = ( self.m[0][1] + self.m[1][0] ) * s
            q.y = 0.5 * s
            q.z = ( self.m[1][2] + self.m[2][1] ) * s
        else:
            s = 1.0 / ( math.sqrt( 1.0 + self.m[2][2] - self.m[0][0] - self.m[1][1] ) * 2.0 )
            q.w = ( self.m[0][1] + self.m[1][0] ) * s
            q.x = ( self.m[0][2] + self.m[2][0] ) * s
            q.y = ( self.m[1][2] + self.m[2][1] ) * s
            q.z = 0.5 * s

        return q


