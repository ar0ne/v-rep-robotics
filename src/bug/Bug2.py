#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

from BugBase import *


class Bug2(BugBase):
    def __init__(self, target_name='target', bot_name='Bot', wheel_speed=1.0):
        BugBase.__init__(self, target_name, bot_name, wheel_speed)
        self.start_target_pos = None
        self.start_bot_pos = None
        self.about = "Algorithm Bug2"

        self.print_about_info()

    def loop(self):

        self._init_values()

        while True:

            self.tick()

            self.stop_move()
            self.read_values()

            # self.calc_lenght_of_robot_track()

            if self.start_bot_pos is None:
                self.start_bot_pos = self.bot_pos
            if self.start_target_pos is None:
                self.start_target_pos = self.target_pos

            self.read_from_sensors()

            self.target_pos.z = self.bot_pos.z = 0.0

            q_rot = Quaternion()
            q_rot.set_from_vector(self.bot_euler_angles.z, Vector3(0.0, 0.0, 1.0))
            self.bot_dir = q_rot.rotate(Vector3(1.0, 0.0, 0.0))

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
            self.target_dir = tmp.rotate(self.bot_dir)

            return

        angle = Utils.angle_between_vectors(self.bot_dir, self.target_pos.minus(self.bot_pos))

        if math.fabs(angle) > 1.0 * self.PI / 180.0:
            vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.WHEEL_SPEED + angle, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.WHEEL_SPEED - angle, vrep.simx_opmode_streaming)
        else:
            vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.WHEEL_SPEED, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.WHEEL_SPEED, vrep.simx_opmode_streaming)

    def action_rotating(self):

        angle = Utils.angle_between_vectors(self.bot_dir, self.target_dir)

        if math.fabs(angle) > 5.0 * self.PI / 180.0:
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
        diff_x = (self.bot_pos.x - self.start_bot_pos.x) / (self.start_target_pos.x - self.start_bot_pos.x)
        diff_y = (self.bot_pos.y - self.start_bot_pos.y) / (self.start_target_pos.y - self.start_bot_pos.y)
        delta = 0.01
        if diff_x - delta < diff_y < diff_x + delta:
            return True
        return False
