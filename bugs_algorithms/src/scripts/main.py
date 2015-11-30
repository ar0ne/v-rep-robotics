#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import math
import time
import vrep
import sys

from support import *

PI = math.pi

def angle_between_vectors(a, b):  # a -> b

    a = a.unitVector()
    b = b.unitVector()
    angle = math.acos( b.dot( a ) )
    if (a.multiply(b)).z > 0.0:
        return -angle
    return angle

class State:
    MOVING = 1
    ROTATING = 2
    ENVELOPING = 3

####################################################

if __name__ == '__main__':
    vrep.simxFinish(-1)

    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if clientID != -1:
        print 'Connected to remote API server'

    else:
        print 'Connection not successful'
        sys.exit('Could not connect')

    errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    # get handle of target robot
    error_code, target_handle = vrep.simxGetObjectHandle(clientID, 'target', vrep.simx_opmode_oneshot_wait)

    # handle of robot
    error_code, bot_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)

    sensor_handles = []  # empty list for handles

    for x in range(1, 16+1):
            errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x), vrep.simx_opmode_oneshot_wait)
            sensor_handles.append(sensor_handle)
            vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)

    obstacleDistStabPID = PIDController(50.0)
    obstacleFolowerPID = PIDController(50.0)
    obstacleDistStabPID.setCoefficients(2, 0, 0.5)
    obstacleFolowerPID.setCoefficients(2, 0, 0)

    state = State.MOVING

    minDetectionDist = 0.0
    maxDetectionDist = 1.0
    detect = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
    motorsPower = 1.0
    withstandDist = 0.5

    targetDir = Vector3()

    error_code, target_pos = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot )

    error_code, bot_pos = vrep.simxGetObjectPosition(clientID, bot_handle, -1, vrep.simx_opmode_oneshot )

    error_code, bot_euler_angles = vrep.simxGetObjectOrientation(clientID, bot_handle, -1, vrep.simx_opmode_streaming )

    while(True):

        time.sleep(0.2)

        vrep.simxSetJointTargetVelocity( clientID, left_motor_handle, 0, vrep.simx_opmode_streaming )
        vrep.simxSetJointTargetVelocity( clientID, right_motor_handle, 0, vrep.simx_opmode_streaming )

        error_code, target_pos = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_buffer)

        error_code, bot_pos = vrep.simxGetObjectPosition(clientID, bot_handle, -1, vrep.simx_opmode_buffer)

        error_code, bot_euler_angles = vrep.simxGetObjectOrientation(clientID, bot_handle, -1, vrep.simx_opmode_buffer)

        goalPos = Vector3(x=target_pos[0], y=target_pos[1], z=target_pos[2])
        botPos = Vector3(x=bot_pos[0], y=bot_pos[1], z=bot_pos[2])
        botEulerAngles = Vector3(x=bot_euler_angles[0], y=bot_euler_angles[1], z=bot_euler_angles[2])

        for i in range(0, 16):

            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(clientID, sensor_handles[i], vrep.simx_opmode_buffer)

            dp = Vector3(x=detected_point[0], y=detected_point[1], z=detected_point[2])
            dist = dp.length()

            if dist < minDetectionDist:
                detect[i] = 0.0
            elif dist > maxDetectionDist or detection_state is False:
                detect[i] = 1.0
            else:
                detect[i] = 1 - (( dist - maxDetectionDist ) / ( minDetectionDist - maxDetectionDist ))

        goalPos.z = botPos.z = 0.0
        qRot = Quaternion()
        qRot.set_from_vector(botEulerAngles.z, Vector3( 0.0, 0.0, 1.0 ))
        botDir = qRot.rotate( Vector3( 1.0, 0.0, 0.0 ) )

        if state == State.MOVING:

            if detect[4] < 0.6:

                state = State.ROTATING
                tmp = Quaternion()
                tmp.set_from_vector(PI / 2.0, Vector3( 0.0, 0.0, 1.0 ))
                targetDir = tmp.rotate( botDir )
                continue

            angle = angle_between_vectors( botDir, goalPos.minus( botPos ))

            if math.fabs( angle ) > 1.0 / 180.0 * PI:
                vrep.simxSetJointTargetVelocity( clientID, left_motor_handle, motorsPower + angle, vrep.simx_opmode_streaming )
                vrep.simxSetJointTargetVelocity( clientID, right_motor_handle, motorsPower - angle, vrep.simx_opmode_streaming )
            else:
                vrep.simxSetJointTargetVelocity( clientID, left_motor_handle, motorsPower, vrep.simx_opmode_streaming )
                vrep.simxSetJointTargetVelocity( clientID, right_motor_handle, motorsPower, vrep.simx_opmode_streaming )

        elif state == State.ROTATING:

            angle = angle_between_vectors( botDir, targetDir )
            if math.fabs(angle) > 5.0 / 180.0 * PI:
                vrep.simxSetJointTargetVelocity( clientID, left_motor_handle, angle, vrep.simx_opmode_streaming )
                vrep.simxSetJointTargetVelocity( clientID, right_motor_handle, -angle, vrep.simx_opmode_streaming )
            else:
                state = State.ENVELOPING

        elif state == State.ENVELOPING:
            tmpDir = Quaternion()
            tmpDir.set_from_vector(PI / 2.0, Vector3( 0.0, 0.0, 1.0 ))
            perpBotDir = tmpDir.rotate(botDir)
            angle = angle_between_vectors( perpBotDir, goalPos.minus(botPos) )

            if math.fabs( angle ) < 5.0 / 180.0 * PI:
                state = State.MOVING
                continue

            delta = detect[7] - detect[8]
            obstacleDist = 0.0

            if delta < 0.0:
                obstacleDist = detect[7] - withstandDist
            else:
                obstacleDist = detect[8] - withstandDist

            uObstacleDistStab = obstacleDistStabPID.output( obstacleDist )
            uObstacleFolower = obstacleFolowerPID.output( delta )

            vrep.simxSetJointTargetVelocity( clientID, left_motor_handle, motorsPower + uObstacleFolower + uObstacleDistStab - ( 1 - detect[4] ), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity( clientID, right_motor_handle, motorsPower - uObstacleFolower - uObstacleDistStab + ( 1 - detect[4] ), vrep.simx_opmode_streaming)
