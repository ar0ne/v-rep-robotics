#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

from BugBase import *


class RobotRoundingState:
    START = 1
    PROCESS = 2
    END = 3

class Bug1(BugBase):
    def __init__(self, target_name='target', bot_name='Bot', wheel_speed=1.0):
        BugBase.__init__(self, target_name, bot_name, wheel_speed)
        self.about = "Algorithm Bug1"
        self.print_about_info()

        self.bot_rounding_state = None
        self.min_dist_to_target = 10000
        self.rounding_start_pos = None

    def loop(self):

        self._init_values()

        while True:

            self.tick()

            self.stop_move()
            self.read_values()

            # self.calc_lenght_of_robot_track()

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

            if self.bot_rounding_state is None:
                self.bot_rounding_state = RobotRoundingState.START
                self.rounding_start_pos = self.bot_pos

    def action_rounding(self):

        if self.bot_rounding_state is RobotRoundingState.START:
            if self.is_cur_pos_near_start_rounding_pos(0.5) is False:
                self.bot_rounding_state = RobotRoundingState.PROCESS

        elif self.bot_rounding_state is RobotRoundingState.PROCESS:
            if self.min_dist_to_target > Utils.distance_between_points(self.bot_pos, self.target_pos):
                self.min_dist_to_target = Utils.distance_between_points(self.bot_pos, self.target_pos)

            if self.is_cur_pos_near_start_rounding_pos(0.2) is True:
                self.bot_rounding_state = RobotRoundingState.END

        elif self.bot_rounding_state is RobotRoundingState.END:

            dist_to_target = Utils.distance_between_points(self.bot_pos, self.target_pos)

            if dist_to_target * 0.95 < self.min_dist_to_target < dist_to_target * 1.05:
                self.state = States.MOVING
                self.bot_rounding_state = None
                self.rounding_start_pos = None
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

    def is_cur_pos_near_start_rounding_pos(self, radius):
        # (x-x0)^2 + (y-y0)^2 <= R^2
        return (self.bot_pos.x - self.rounding_start_pos.x) ** 2 + (self.bot_pos.y - self.rounding_start_pos.y) <= radius ** 2
