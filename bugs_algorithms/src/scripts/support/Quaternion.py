#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import math
from Vector3 import *
from EulerAngles import *
from Matrix3 import *

class Quaternion:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def set_from_vector(self, angle, dir):
        halfAngle = angle / 2.0
        sinHalfAngle = math.sin(halfAngle)
        self.w = math.cos(halfAngle)
        self.x = sinHalfAngle * dir.x
        self.y = sinHalfAngle * dir.y
        self.z = sinHalfAngle * dir.z

    def multiply(self, q):
        res = Quaternion()
        res.w = self.w * q.w - self.x * q.x - self.y * q.y - self.z * q.z
        res.x = self.w * q.x + self.x * q.w + self.y * q.z - self.z * q.y
        res.y = self.w * q.y - self.x * q.z + self.y * q.w + self.z * q.x
        res.z = self.w * q.z + self.x * q.y - self.y * q.x + self.z * q.w
        return res

    def multiply_to_number(self, num):
        res = Quaternion()
        res.w = self.w * num
        res.x = self.x * num
        res.y = self.y * num
        res.z = self.z * num
        return res

    def plus(self, q):
        res = Quaternion()
        res.w = self.w + q.w
        res.x = self.x + q.x
        res.y = self.y + q.y
        res.z = self.z + q.z

        return res

    def get_by_index(self, index):
        if index is 0:
            return self.w
        elif index is 1:
            return self.x
        elif index is 2:
            return self.y
        elif index is 3:
            return self.z

    def dot(self, q):
        return self.w * q.w + self.x * q.x + self.y * q.y + self.z * q.z

    def norm(self):
        return self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2

    def magnitude(self):
        return math.sqrt(self.norm())

    def conjugate(self):
        res = Quaternion()
        res.w = self.w
        res.x = - self.x
        res.y = - self.y
        res.z = - self.z

        return res

    def inverse(self):
        res = Quaternion()
        n = self.norm()
        res.w = self.w / n
        res.x = - self.x / n
        res.y = - self.y / n
        res.z = -self.z / n
        return res

    def rotate(self, v):
        vectQuad = Quaternion(w=0.0, x=v.x, y=v.y, z=v.z)
        q1 = self.multiply(vectQuad)
        res = q1.multiply(self.inverse())
        return Vector3(x=res.x, y=res.y, z=res.z)

    def toMatrix3(self):
        matr = Matrix3()

        xx = self.x ** 2
        xy = self.x * self.y
        xz = self.x * self.z
        xw = self.x * self.w

        yy = self.y ** 2
        yz = self.y * self.z
        yw = self.y * self.w

        zz = self.z ** 2
        zw = self.z * self.w

        matr.m[0][0] = 1 - 2 * (yy + zz)
        matr.m[0][1] = 2 * (xy - zw)
        matr.m[0][2] = 2 * (xz + yw)

        matr.m[1][0] = 2 * (xy + zw)
        matr.m[1][1] = 1 - 2 * (xx + zz)
        matr.m[1][2] = 2 * (yz - xw)

        matr.m[2][0] = 2 * (xz - yw)
        matr.m[2][1] = 2 * (yz + xw)
        matr.m[2][2] = 1 - 2 * (xx + yy)

        return matr

    def toEulerAngles(self):
        angles = EulerAngles()

        norm = self.norm()
        test = self.x * self.y + self.z * self.w
        absTest = math.fabs(test)
        if absTest < 0.005 * norm:
            if test > 0:
                # singularity at north pole
                angles.yaw = -2.0 * math.atan2(self.x, self.w)
                angles.roll = 0
                angles.pitch = math.pi / 2.0

            else:
                # singularity at south pole
                angles.yaw = 2.0 * math.atan2(self.x, self.w)
                angles.roll = 0
                angles.pitch = - math.pi / 2.0

            return angles

        m = 2.0 * (self.x * self.z - self.w * self.y)

        angles.roll = math.atan2(2.0 * (self.y * self.z + self.w * self.x), 2.0 * (self.w ** 2 + self.z ** 2) - 1.0)
        angles.pitch = - math.atan( m / math.sqrt(1.0 - m ** 2))
        angles.yaw = math.atan2(2.0 * (self.x * self.y + self.w * self.z), 2.0 * (self.w ** 2 + self.x ** 2) - 1.0)

        return angles
