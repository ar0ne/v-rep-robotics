#!/usr/bin/env python
# -*- coding: utf-8 -*-


import vrep
from BugBase import BugBase

__author__ = 'ar1'


class Bug1(BugBase):
    def __init__(self, target_name='target', bot_name='Bot', wheel_speed=1.0):
        BugBase.__init__(self, target_name, bot_name, wheel_speed)

    def loop(self):
        pass