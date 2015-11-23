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

        self._client_id = self._init_client_id()
        self._left_wheel_handle, self._right_wheel_handle = self._init_wheels_handles()
        self._robot_handle = self._init_robot_handle()
        self._sensors_handle, self._sensor_values = self._init_sensors_handle()
        self._target_handle = self._init_target_handle()
        self._target_position = self._init_target_position()
        self._robot_orientation = self._init_robot_orientation()

        # orientation of all the sensors:
        self._sensor_loc = np.array([-PI/2, -50/180.0 * PI, -30/180.0 * PI, -10/180.0 * PI, 10/180.0 * PI,
                                     30/180.0 * PI, 50/180.0 * PI, PI/2, PI/2, 130/180.0 * PI, 150/180.0 * PI,
                                     170/180.0 * PI, -170/180.0 * PI, -150/180.0 * PI, -130/180.0 * PI, -PI/2])

        self._sleep_time = 0.2

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

            print "OLOLO"

            self.sleep()

            # other stuff

    def move(self, v_l = 1, v_r = 1):
        error_code = vrep.simxSetJointTargetVelocity(self._client_id, self._left_wheel_handle,  v_l, vrep.simx_opmode_streaming)
        error_code = vrep.simxSetJointTargetVelocity(self._client_id, self._right_wheel_handle, v_r, vrep.simx_opmode_streaming)

    def step(self):
        #
        # Check if we detect something
        #
        #
        return 1,1

    def _init_target_position(self):
        error_code, target_position = vrep.simxGetObjectPosition(self._client_id, self._target_handle, self._robot_handle, vrep.simx_opmode_streaming)
        return target_position

    def _init_robot_orientation(self):
        error_code, euler_angles = vrep.simxGetObjectOrientation(self._client_id, self._robot_handle, -1, vrep.simx_opmode_streaming)
        return euler_angles

    def read_target_position(self):
        error_code, target_position = vrep.simxGetObjectPosition(self._client_id, self._target_handle, self._robot_handle, vrep.simx_opmode_buffer)
        return target_position

    def read_robot_orientation(self):
        error_code, euler_angles = vrep.simxGetObjectOrientation(self._client_id, self._robot_handle, -1, vrep.simx_opmode_buffer)
        return euler_angles

    def _rotate_robot_to_target(self):

        angles = self.read_robot_orientation()

        norm = np.linalg.norm(angles)

        while(norm > 0.02):

            angles = self.read_robot_orientation()

            norm = np.linalg.norm(angles)

            self.move(0.05, -0.05)

            print(norm)

            self.sleep(0.01)

        self.stop_move()

    def read_data_from_face_sensors(self):
        #
        # Read data from first 8 sensors
        #
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
        errorCode = vrep.simxSetJointTargetVelocity(self._client_id, self._left_wheel_handle,  0, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(self._client_id, self._right_wheel_handle, 0, vrep.simx_opmode_streaming)

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

    robot.sleep()

    robot.stop_move()

    robot._rotate_robot_to_target()

    robot.stop_move()

    # robot.update()