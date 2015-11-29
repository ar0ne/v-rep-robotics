#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'


import vrep
import sys
import time
import numpy as np
import math

PI = math.pi

class Robot:
    def __init__(self, host='127.0.0.1', port=19999):

        self._hostname = host
        self._port = port

        self._sleep_time = 0.05

        self._client_id = self._init_client_id()
        self._left_wheel_handle, self._right_wheel_handle = self._init_wheels_handles()
        self._robot_handle = self._init_robot_handle()
        self._sensors_handle, self._sensor_values = self._init_sensors_handle()
        self._target_handle = self._init_target_handle()
        self._target_position = self._init_target_position()
        self._robot_orientation = self._init_robot_orientation()
        self._robot_position = self._init_robot_position()

        # orientation of all the sensors:
        self._sensor_loc = np.array([-PI/2, -50/180.0 * PI, -30/180.0 * PI, -10/180.0 * PI, 10/180.0 * PI,
                                     30/180.0 * PI, 50/180.0 * PI, PI/2, PI/2, 130/180.0 * PI, 150/180.0 * PI,
                                     170/180.0 * PI, -170/180.0 * PI, -150/180.0 * PI, -130/180.0 * PI, -PI/2])

    def _init_client_id(self):

        client_id = vrep.simxStart(self._hostname, self._port, True, True, 5000, 5)

        if client_id == -1:
            print 'Connection not successful'
            sys.exit('Could not connect')
        else:
            print 'Connected to remote API server'

        return client_id

    def update(self):
        while(True):
            v_l, v_r = self.step()

            self.move(v_l, v_r)

            # other stuff

    def move(self, v_l = 1, v_r = 1):
        error_code = vrep.simxSetJointTargetVelocity(self._client_id, self._left_wheel_handle,  v_l, vrep.simx_opmode_streaming)
        error_code = vrep.simxSetJointTargetVelocity(self._client_id, self._right_wheel_handle, v_r, vrep.simx_opmode_streaming)
        self.sleep(0.01)
        self.stop_move()

    def step(self):
        #
        # Check if we detect something
        #
        #
        return 1, 1

    def _init_target_position(self):
        self.sleep()
        error_code, target_position = vrep.simxGetObjectPosition(self._client_id, self._target_handle, -1, vrep.simx_opmode_streaming)
        return target_position

    def read_target_position(self):
        self.sleep()
        error_code, target_position = vrep.simxGetObjectPosition(self._client_id, self._target_handle, -1, vrep.simx_opmode_buffer)
        return target_position

    def _init_robot_position(self):
        self.sleep()
        error_code, robot_position = vrep.simxGetObjectPosition(self._client_id, self._robot_handle, -1, vrep.simx_opmode_streaming)
        return robot_position

    def _init_robot_orientation(self):
        self.sleep()
        error_code, euler_angles = vrep.simxGetObjectOrientation(self._client_id, self._robot_handle, -1, vrep.simx_opmode_streaming)
        return euler_angles

    def read_robot_orientation(self):
        self.sleep()
        error_code, euler_angles = vrep.simxGetObjectOrientation(self._client_id, self._robot_handle, -1, vrep.simx_opmode_buffer)
        return euler_angles

    def read_robot_position(self):
        self.sleep()
        error_code, robot_position = vrep.simxGetObjectPosition(self._client_id, self._robot_handle, -1, vrep.simx_opmode_buffer)
        return robot_position

    def _rotate_to_zero(self):

        print("Rotate to Zero degree...")

        self.sleep()

        rot1, rot2 = 0, 0

        min_speed = 0.1
        delta = 0.005

        ornt = self.read_robot_orientation()
        norm = np.linalg.norm(ornt)

        if ornt[0] > 0:
            rot1, rot2 = min_speed, -min_speed  # поворачивать против часовой
        else:
            rot1, rot2 = -min_speed, min_speed

        while( delta < norm or norm > delta):

            angles = self.read_robot_orientation()

            norm = np.linalg.norm(angles)

            self.move(rot1, rot2)

        self.stop_move()

    def _rotate_at_90_degrees(self):

        print("Rotate at 90 degrees...")

        foo = self.read_robot_orientation()
        ornt = np.linalg.norm(foo)
        norm = ornt
        rad_90 = self.deg2rad(90)
        rad_180 = self.deg2rad(180)

        if foo[0] > 0:
            if ornt > rad_90:
                tmp = rad_180 - ornt
                ornt = rad_90 - tmp
            else:
                ornt = rad_90 + ornt
        else:
            if ornt < rad_90:
                ornt = rad_90 - ornt
            else:
                tmp = rad_180 - ornt
                ornt = rad_180 - (rad_90 - tmp)

        desp = 0.005
        speed = 0.02
        min = ornt - desp
        max = ornt + desp
        while not (norm > min and norm < max):
            self.move(-speed, speed)
            norm = np.linalg.norm(self.read_robot_orientation())
            # print(norm, ornt)

    def rotate_robot_to_target(self):

        print("Rotate to target...")

        self._rotate_to_zero()

        alpha = self._get_angles_between_robot_and_target()

        print("ALPHA: " + str(alpha))

        sign = True  # right side

        rad_alpha = 0

        if alpha < 180:
            sign = True
            rad_alpha = self.deg2rad(alpha)
        elif alpha >= 180:
            sign = False
            rad_alpha = self.deg2rad(alpha - 180)

        norm = 0
        desp = 0.005
        speed = 0.02
        min = rad_alpha - desp
        max = rad_alpha + desp

        while not (norm > min and norm < max):
            if sign:
                self.move(speed, -speed)
            else:
                self.move(-speed, speed)

            norm = np.linalg.norm(self.read_robot_orientation())
            # print(norm, rad_alpha)

    def _get_angles_between_robot_and_target(self):

        self.sleep()

        x0, y0, _ = self.read_robot_position()
        x1, y1, _ = self.read_target_position()

        delta_y, delta_x = 0, 0

        if y0 > y1 and x0 > x1:
            delta_y = y1 - y0
            delta_x = x1 - x0
        else:
            delta_y = y0 - y1
            delta_x = x0 - x1

        alpha = math.atan2(delta_y, delta_x) / PI * 180

        if alpha < 0:
            alpha += 360

        return alpha

    def rad2deg(self, rad):
        return rad * 180 / PI

    def deg2rad(self, deg):
        return deg * PI / 180

    def read_data_from_face_sensors(self):
        #
        # Read data from first 8 sensors
        #
        sensor_val = np.array([])

        for x in xrange(1, 8 + 1):
            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(self._client_id, self._sensors_handle[x], vrep.simx_opmode_buffer)
            sensor_val = np.append(sensor_val, np.linalg.norm(detected_point))
        return sensor_val

    def _init_robot_handle(self):
        error_code, robot_handle = vrep.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
        return robot_handle

    def _init_wheels_handles(self):
        error_code, left_motor_handle  = vrep.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        error_code, right_motor_handle = vrep.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        # @TODO: add checks for error codes
        return left_motor_handle, right_motor_handle

    def get_left_wheel_handle(self):
        return self._left_wheel_handle

    def get_right_wheel_handle(self):
        return self._right_wheel_handle

    def get_client_id(self):
        return self._client_id

    def _init_sensors_handle(self):
        sensor_h = []  # empty list for handles
        sensor_val = np.array([])  # empty array for sensor measurements

        # for loop to retrieve sensor arrays and initiate sensors
        for x in xrange(1, 16 + 1):
            error_code, sensor_handle = vrep.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(x), vrep.simx_opmode_oneshot_wait)
            sensor_h.append(sensor_handle)  # keep list of handles
            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(self._client_id, sensor_handle, vrep.simx_opmode_streaming)
            sensor_val = np.append(sensor_val, np.linalg.norm(detected_point))  # get list of values

        return sensor_h, sensor_val

    def stop_move(self):
        error_code = vrep.simxSetJointTargetVelocity(self._client_id, self._left_wheel_handle,  0, vrep.simx_opmode_streaming)
        error_code = vrep.simxSetJointTargetVelocity(self._client_id, self._right_wheel_handle, 0, vrep.simx_opmode_streaming)

    def sleep(self, sec=None):
        if sec is None:
            sec = self._sleep_time
        time.sleep(sec)

    def _init_target_handle(self):
        error_code, target_h = vrep.simxGetObjectHandle(self._client_id, 'target', vrep.simx_opmode_oneshot_wait)
        return target_h

######################################################
if __name__ == '__main__':

    vrep.simxFinish(-1)

    robot = Robot()

    robot.stop_move()

    robot.rotate_robot_to_target()

    # robot._rotate_at_90_degrees()

    robot.stop_move()

    print("END")

    # robot.update()
