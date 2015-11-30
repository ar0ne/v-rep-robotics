#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import math
import time
import vrep
import sys

from support import *

PI = math.pi

class State:
    MOVING = 1
    ROTATING = 2
    ENVELOPING = 3

class Bug2:
    def __init__(self):
        self._init_client_id()
        self._init_handles()
        self._init_sensor_handles()

        self.state = State.MOVING

        self.min_detection_dist = 0.0
        self.max_detection_dist = 1.0
        self.detect = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
        self.motors_power = 1.0
        self.withstand_dist = 0.5

    def _init_client_id(self):
        vrep.simxFinish(-1)

        self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if self.client_id != -1:
            print 'Connected to remote API server'

        else:
            print 'Connection not successful'
            sys.exit('Could not connect')

    def angle_between_vectors(self, a, b):  # a -> b

        a = a.unitVector()
        b = b.unitVector()
        angle = math.acos( b.dot( a ) )
        if (a.multiply(b)).z > 0.0:
            return -angle
        return angle

    def _init_handles(self):
        self._init_wheels_handle()

        self._init_target_handle()

        self._init_robot_handle()

    def _init_robot_handle(self):
        # handle of robot
        error_code, self.bot_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx',
                                                               vrep.simx_opmode_oneshot_wait)

    def _init_target_handle(self):
        # get handle of target robot
        error_code, self.target_handle = vrep.simxGetObjectHandle(self.client_id, 'target',
                                                                  vrep.simx_opmode_oneshot_wait)

    def _init_wheels_handle(self):
        # get handles of robot wheels
        errorCode, self.left_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor',
                                                                     vrep.simx_opmode_oneshot_wait)
        errorCode, self.right_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor',
                                                                      vrep.simx_opmode_oneshot_wait)

    def _init_sensor_handles(self):

        self.sensor_handles = []  # empty list for handles

        for x in range(1, 16 + 1):
            error_code, sensor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(x), vrep.simx_opmode_oneshot_wait)
            self.sensor_handles.append(sensor_handle)
            vrep.simxReadProximitySensor(self.client_id, sensor_handle, vrep.simx_opmode_streaming)

    def _init_values(self):

        error_code, self.target_pos = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_oneshot )

        error_code, self.bot_pos = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_oneshot )

        error_code, self.bot_euler_angles = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming )

    def read_values(self):

        error_code, self.target_pos = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_buffer)

        error_code, self.bot_pos = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_buffer)

        error_code, self.bot_euler_angles = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_buffer)

    def stop_move(self):
        vrep.simxSetJointTargetVelocity( self.client_id, self.left_motor_handle, 0, vrep.simx_opmode_streaming )
        vrep.simxSetJointTargetVelocity( self.client_id, self.right_motor_handle, 0, vrep.simx_opmode_streaming )

    def read_from_sensors(self):

        for i in range(0, 16):

            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(self.client_id, self.sensor_handles[i], vrep.simx_opmode_buffer)

            dp = Vector3(x=detected_point[0], y=detected_point[1], z=detected_point[2])
            dist = dp.length()

            if dist < self.min_detection_dist:
                self.detect[i] = 0.0
            elif dist > self.max_detection_dist or detection_state is False:
                self.detect[i] = 1.0
            else:
                self.detect[i] = 1.0 - ((dist - self.max_detection_dist) / (self.min_detection_dist - self.max_detection_dist))

    def loop(self):

        self._init_values()

        self.obstacle_dist_stab_PID = PIDController(50.0)
        self.obstacle_follower_PID = PIDController(50.0)
        self.obstacle_dist_stab_PID.setCoefficients(2, 0, 0.5)
        self.obstacle_follower_PID.setCoefficients(2, 0, 0)

        targetDir = Vector3()

        while True:
            time.sleep(0.2)

            self.stop_move()
            self.read_values()

            self.goalPos = Vector3(x=self.target_pos[0], y=self.target_pos[1], z=self.target_pos[2])
            self.botPos = Vector3(x=self.bot_pos[0], y=self.bot_pos[1], z=self.bot_pos[2])
            self.botEulerAngles = Vector3(x=self.bot_euler_angles[0], y=self.bot_euler_angles[1], z=self.bot_euler_angles[2])

            self.read_from_sensors()

            self.goalPos.z = self.botPos.z = 0.0
            qRot = Quaternion()
            qRot.set_from_vector(self.botEulerAngles.z, Vector3( 0.0, 0.0, 1.0 ))
            self.botDir = qRot.rotate( Vector3( 1.0, 0.0, 0.0 ) )

            if self.state == State.MOVING:
                self.action_moving()
            elif self.state == State.ROTATING:
                self.action_rotating()
            elif self.state == State.ENVELOPING:
                self.action_enveloping()

    def action_moving(self):

        if self.detect[4] < 0.6:

            self.state = State.ROTATING
            tmp = Quaternion()
            tmp.set_from_vector(PI / 2.0, Vector3( 0.0, 0.0, 1.0 ))
            self.targetDir = tmp.rotate(self.botDir)
            return

        angle = self.angle_between_vectors( self.botDir, self.goalPos.minus( self.botPos ))

        if math.fabs( angle ) > 1.0 / 180.0 * PI:
            vrep.simxSetJointTargetVelocity( self.client_id, self.left_motor_handle, self.motors_power + angle, vrep.simx_opmode_streaming )
            vrep.simxSetJointTargetVelocity( self.client_id, self.right_motor_handle, self.motors_power - angle, vrep.simx_opmode_streaming )
        else:
            vrep.simxSetJointTargetVelocity( self.client_id, self.left_motor_handle, self.motors_power, vrep.simx_opmode_streaming )
            vrep.simxSetJointTargetVelocity( self.client_id, self.right_motor_handle, self.motors_power, vrep.simx_opmode_streaming )

    def action_rotating(self):

        angle = self.angle_between_vectors( self.botDir, self.targetDir)

        if math.fabs(angle) > 5.0 / 180.0 * PI:
            vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, angle, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, -angle, vrep.simx_opmode_streaming)
        else:
            self.state = State.ENVELOPING

    def action_enveloping(self):
        tmpDir = Quaternion()
        tmpDir.set_from_vector(PI / 2.0, Vector3( 0.0, 0.0, 1.0 ))
        perpBotDir = tmpDir.rotate(self.botDir)
        angle = self.angle_between_vectors( perpBotDir, self.goalPos.minus(self.botPos) )

        if math.fabs( angle ) < 5.0 / 180.0 * PI:
            self.state = State.MOVING
            return

        delta = self.detect[7] - self.detect[8]

        if delta < 0.0:
            obstacle_dist = self.detect[7] - self.withstand_dist
        else:
            obstacle_dist = self.detect[8] - self.withstand_dist

        u_obstacle_dist_stab = self.obstacle_dist_stab_PID.output(obstacle_dist)
        u_obstacle_follower = self.obstacle_follower_PID.output(delta)

        vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, self.motors_power + u_obstacle_follower + u_obstacle_dist_stab - (1 - self.detect[4]), vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.motors_power - u_obstacle_follower - u_obstacle_dist_stab + (1 - self.detect[4]), vrep.simx_opmode_streaming)


####################################################

if __name__ == '__main__':

    bug2 = Bug2()

    bug2.loop()




