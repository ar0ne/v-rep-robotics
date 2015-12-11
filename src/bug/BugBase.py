#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import time
import sys
import numpy as np

import vrep
from libs import *


class BugBase:
    def __init__(self, target_name='target', bot_name='Bot', wheel_speed=1.0):

        self.state = States.MOVING

        self.MIN_DETECTION_DIST = 0.0
        self.MAX_DETECTION_DIST = 1.0

        self.TARGET_NAME = target_name
        self.BOT_NAME = bot_name
        self.WHEEL_SPEED = wheel_speed
        self.INDENT_DIST = 0.5

        self.SLEEP_TIME = 0.2
        self.PI = math.pi

        self.obstacle_dist_stab_PID = None
        self.obstacle_follower_PID = None

        self.bot_dir = None
        self.bot_pos = None
        self.bot_euler_angles = None

        self.target_pos = None
        self.target_dir = np.zeros(3)

        self.track_length = 0.0
        self.prev_pos = None

        self.detect = np.zeros(16)

        self._init_client_id()
        self._init_handles()
        self._init_sensor_handles()

        self.obstacle_dist_stab_PID = PIDController(50.0)
        self.obstacle_follower_PID = PIDController(50.0)
        self.obstacle_dist_stab_PID.set_coefficients(2, 0, 0.5)
        self.obstacle_follower_PID.set_coefficients(2, 0, 0)

        self.about = ''

    def _init_client_id(self):
        vrep.simxFinish(-1)

        self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if self.client_id != -1:
            print 'Connected to remote API server'

        else:
            print 'Connection not successful'
            sys.exit('Could not connect')

    def _init_handles(self):

        self._init_wheels_handle()

        self._init_target_handle()

        self._init_robot_handle()

    def _init_robot_handle(self):
        # handle of robot
        error_code, self.bot_handle = vrep.simxGetObjectHandle(self.client_id, self.BOT_NAME, vrep.simx_opmode_oneshot_wait)

    def _init_target_handle(self):
        # get handle of target robot
        error_code, self.target_handle = vrep.simxGetObjectHandle(self.client_id, self.TARGET_NAME, vrep.simx_opmode_oneshot_wait)

    def _init_wheels_handle(self):
        # get handles of robot wheels
        error_code, self.left_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        error_code, self.right_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    def _init_sensor_handles(self):

        self.sensor_handles = []  # empty list for handles

        for x in range(1, 16 + 1):
            error_code, sensor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(x), vrep.simx_opmode_oneshot_wait)

            self.sensor_handles.append(sensor_handle)

            vrep.simxReadProximitySensor(self.client_id, sensor_handle, vrep.simx_opmode_streaming)

    def _init_values(self):

        error_code, _ = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_oneshot)

        error_code, _ = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_oneshot)

        error_code, _ = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)

    def read_values(self):

        error_code, target_pos = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_streaming)
        self.target_pos = Vector3(x=target_pos[0], y=target_pos[1], z=target_pos[2])

        error_code, bot_pos = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)
        self.bot_pos = Vector3(x=bot_pos[0], y=bot_pos[1], z=bot_pos[2])

        error_code, bot_euler_angles = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)
        self.bot_euler_angles = Vector3(x=bot_euler_angles[0], y=bot_euler_angles[1], z=bot_euler_angles[2])

    def stop_move(self):
        error_code = vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  0, vrep.simx_opmode_streaming)
        error_code = vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, 0, vrep.simx_opmode_streaming)

    def read_from_sensors(self):

        for i in range(0, 16):

            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(self.client_id, self.sensor_handles[i], vrep.simx_opmode_streaming)

            dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)

            if dist < self.MIN_DETECTION_DIST:
                self.detect[i] = self.MIN_DETECTION_DIST
            elif dist > self.MAX_DETECTION_DIST or detection_state is False:
                self.detect[i] = self.MAX_DETECTION_DIST
            else:
                self.detect[i] = self.MAX_DETECTION_DIST - ((dist - self.MAX_DETECTION_DIST) / (self.MIN_DETECTION_DIST - self.MAX_DETECTION_DIST))

    def print_about_info(self):
        print("Algorithm: {0}\nTarget name: {1}\nBot name: {2}\nSpeed of wheel: {3}".format(self.about, self.TARGET_NAME, self.BOT_NAME, self.WHEEL_SPEED))

    def calc_lenght_of_robot_track(self):

        if self.prev_pos is None:
            self.prev_pos = self.bot_pos

        self.track_length += Utils.distance_between_points(self.prev_pos, self.bot_pos)
        self.prev_pos = self.bot_pos

        print(self.track_length)

    def tick(self):
        time.sleep(self.SLEEP_TIME)

    def loop(self):
        pass

    def action_moving(self):
        pass

    def action_rotating(self):
        pass

    def action_rounding(self):
        pass





