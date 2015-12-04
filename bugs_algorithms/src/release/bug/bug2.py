#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import vrep
import numpy as np

from bug_base import BugBase
from libs import *


class Bug2(BugBase):
    def __init__(self, target_name='target', bot_name='Bot', wheel_speed=1.0):
        BugBase.__init__(self, target_name, bot_name, wheel_speed)
        self.start_target_pos = None
        self.start_bot_pos = None

    def loop(self):

        self._init_values()

        self.obstacle_dist_stab_PID = PIDController(50.0)
        self.obstacle_follower_PID = PIDController(50.0)
        self.obstacle_dist_stab_PID.setCoefficients(2, 0, 0.5)
        self.obstacle_follower_PID.setCoefficients(2, 0, 0)

        self.targetDir = np.zeros(3)

        while True:

            self.tick()

            self.stop_move()
            self.read_values()

            self.targetPos = Vector3(x=self.target_pos[0], y=self.target_pos[1], z=self.target_pos[2])

            self.botPos = Vector3(x=self.bot_pos[0], y=self.bot_pos[1], z=self.bot_pos[2])

            self.botEulerAngles = Vector3(x=self.bot_euler_angles[0], y=self.bot_euler_angles[1], z=self.bot_euler_angles[2])

            if self.start_bot_pos is None:
                self.start_bot_pos = self.botPos
            if self.start_target_pos is None:
                self.start_target_pos = self.targetPos

            self.read_from_sensors()

            self.targetPos.z = self.botPos.z = 0.0
            qRot = Quaternion()
            qRot.set_from_vector(self.botEulerAngles.z, Vector3(0.0, 0.0, 1.0))
            self.botDir = qRot.rotate(Vector3(1.0, 0.0, 0.0))

            if self.state == States.MOVING:
                self.action_moving()
            elif self.state == States.ROTATING:
                self.action_rotating()
            elif self.state == States.ROUNDING:
                self.action_rounding()

    def action_moving(self):

        if self.detect[4] < 0.6:

            self.state = States.ROTATING
            tmp = Quaternion()
            tmp.set_from_vector(self.PI / 2.0, Vector3(0.0, 0.0, 1.0))
            self.targetDir = tmp.rotate(self.botDir)

            return

        angle = Utils.angle_between_vectors(self.botDir, self.targetPos.minus(self.botPos))

        if math.fabs(angle) > 1.0 / 180.0 * self.PI:
            vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.WHEEL_SPEED + angle, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.WHEEL_SPEED - angle, vrep.simx_opmode_streaming)
        else:
            vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.WHEEL_SPEED, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.WHEEL_SPEED, vrep.simx_opmode_streaming)

    def action_rotating(self):

        angle = Utils.angle_between_vectors(self.botDir, self.targetDir)

        if math.fabs(angle) > 5.0 / 180.0 * self.PI:
            vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,   angle, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, -angle, vrep.simx_opmode_streaming)
        else:
            self.state = States.ROUNDING

    def action_rounding(self):

        if self.is_bot_on_the_constant_direction():
            self.state = States.MOVING
            return

        delta = self.detect[7] - self.detect[8]

        if delta < 0.0:
            obstacle_dist = self.detect[7] - self.INDENT_DIST
        else:
            obstacle_dist = self.detect[8] - self.INDENT_DIST

        u_obstacle_dist_stab = self.obstacle_dist_stab_PID.output(obstacle_dist)
        u_obstacle_follower = self.obstacle_follower_PID.output(delta)

        vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.WHEEL_SPEED + u_obstacle_follower + u_obstacle_dist_stab - (1 - self.detect[4]), vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.WHEEL_SPEED - u_obstacle_follower - u_obstacle_dist_stab + (1 - self.detect[4]), vrep.simx_opmode_streaming)

    def is_bot_on_the_constant_direction(self):
        # (x-x1)/(x2-x1) = (y-y1)/(y2-y1).
        diff_x = (self.botPos.x - self.start_bot_pos.x) / (self.start_target_pos.x - self.start_bot_pos.x)
        diff_y = (self.botPos.y - self.start_bot_pos.y) / (self.start_target_pos.y - self.start_bot_pos.y)
        delta = 0.01
        if diff_x - delta < diff_y < diff_x + delta:
            return True
        return False
        