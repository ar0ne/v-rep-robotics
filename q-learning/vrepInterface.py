#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import time
import numpy as np
import vrep
import config
import math

# V-REP data transmission modes:
WAIT = vrep.simx_opmode_oneshot_wait
ONESHOT = vrep.simx_opmode_oneshot
STREAMING = vrep.simx_opmode_streaming
BUFFER = vrep.simx_opmode_buffer
BLOCKING = vrep.simx_opmode_blocking

if config.wait_response:
    MODE_INI = WAIT
    MODE = WAIT
else:
    MODE_INI = STREAMING
    MODE = BUFFER

robotID = -1
ultraID = [-1] * config.n_ultra
rewardRefID = -1
goalID = -1
left_motorID = -1
right_motorID = -1
clientID = -1

wall0_collisionID = -1
wall1_collisionID = -1
wall2_collisionID = -1
wall3_collisionID = -1

target_collisionID = -1

distance = np.full(config.n_ultra, -1, dtype=np.float64)  # distance from ultrasonic sensors
pos = np.full(3, -1, dtype=np.float64)  # Pose 2d base: x(m), y(m), theta(rad)
reward_ref = np.full(1, -1, dtype=np.float64)  # reward reference: distance between goal and robot


def show_msg(message):
    """ send a message for printing in V-REP """
    vrep.simxAddStatusbarMessage(clientID, message, WAIT)
    return


def connect():
    """ Connect to the simulator"""
    ip = '127.0.0.1'
    port = 19997
    vrep.simxFinish(-1)  # just in case, close all opened connections
    global clientID
    clientID = vrep.simxStart(ip, port, True, True, 3000, 5)
    # Connect to V-REP
    if clientID == -1:
        import sys
        sys.exit('\nV-REP remote API server connection failed (' + ip + ':' +
                 str(port) + '). Is V-REP running?')
    print('Connected to Remote API Server')  # show in the terminal
    show_msg('Python: Hello')  # show in the VREP
    time.sleep(config.SLEEP_TIME)
    return


def start():
    """ Start the simulation (force stop and setup)"""
    global clientID
    stop()
    setup_devices()
    vrep.simxStartSimulation(clientID, ONESHOT)
    time.sleep(config.SLEEP_TIME)
    # Solve a rare bug in the simulator by repeating:
    setup_devices()
    vrep.simxStartSimulation(clientID, ONESHOT)
    time.sleep(config.SLEEP_TIME)


def stop():
    """ Stop the simulation """
    global clientID
    vrep.simxStopSimulation(clientID, ONESHOT)
    time.sleep(config.SLEEP_TIME)


def setup_devices():
    """ Assign the devices from the simulator to specific IDs """
    global robotID, left_motorID, right_motorID, ultraID, rewardRefID, goalID, wall0_collisionID, wall1_collisionID, wall2_collisionID, wall3_collisionID, target_collisionID
    # res: result (1(OK), -1(error), 0(not called))
    # robot
    res, robotID = vrep.simxGetObjectHandle(clientID, 'robot#', WAIT)
    # motors
    res, left_motorID = vrep.simxGetObjectHandle(clientID, 'leftMotor#', WAIT)
    res, right_motorID = vrep.simxGetObjectHandle(clientID, 'rightMotor#', WAIT)
    # ultrasonic sensors
    for idx, item in enumerate(config.ultra_distribution):
        res, ultraID[idx] = vrep.simxGetObjectHandle(clientID, item, WAIT)
    # reward reference distance object
    res, rewardRefID = vrep.simxGetDistanceHandle(clientID, 'Distance#', WAIT)
    # if res == vrep.simx_return_ok:  # [debug]
    #    print("vrep.simxGetDistanceHandle executed fine")

    # goal reference object
    res, goalID = vrep.simxGetObjectHandle(clientID, 'Dummy#', WAIT)
    # collision object
    res, target_collisionID = vrep.simxGetCollisionHandle(clientID, "targetCollision#", BLOCKING)
    res, wall0_collisionID = vrep.simxGetCollisionHandle(clientID, "wall0#", BLOCKING)
    res, wall1_collisionID = vrep.simxGetCollisionHandle(clientID, "wall1#", BLOCKING)
    res, wall2_collisionID = vrep.simxGetCollisionHandle(clientID, "wall2#", BLOCKING)
    res, wall3_collisionID = vrep.simxGetCollisionHandle(clientID, "wall3#", BLOCKING)

    # start up devices

    # wheels
    vrep.simxSetJointTargetVelocity(clientID, left_motorID, 0, STREAMING)
    vrep.simxSetJointTargetVelocity(clientID, right_motorID, 0, STREAMING)
    # pose
    vrep.simxGetObjectPosition(clientID, robotID, -1, MODE_INI)
    vrep.simxGetObjectOrientation(clientID, robotID, -1, MODE_INI)

    # reading-related function initialization according to the recommended operationMode
    for i in ultraID:
        vrep.simxReadProximitySensor(clientID, i, STREAMING)
    vrep.simxReadDistance(clientID, rewardRefID, STREAMING)
    vrep.simxReadCollision(clientID, wall0_collisionID, STREAMING)
    vrep.simxReadCollision(clientID, wall1_collisionID, STREAMING)
    vrep.simxReadCollision(clientID, wall2_collisionID, STREAMING)
    vrep.simxReadCollision(clientID, wall3_collisionID, STREAMING)
    vrep.simxReadCollision(clientID, target_collisionID, STREAMING)


def get_ultra_distance():
    """ return distances measured by ultrasonic sensors(m) """
    global distance, flag
    state = [False, False]
    flag = 0
    for i, item in enumerate(ultraID):
        _, state[i], detectedPoint, _, _ = vrep.simxReadProximitySensor(clientID, item, BUFFER)
        if state[i] == True:
            distance[i] = math.sqrt(detectedPoint[0] ** 2 + detectedPoint[1] ** 2 + detectedPoint[2] ** 2)
            # discretization
            distance[i] = np.floor((np.floor(distance[i] / (config.grid_width / 2)) + 1) / 2) * config.grid_width
            distance[i] = "{0:.2f}".format(round(distance[i], config.PRECISION))
        else:
            distance[i] = -1
            flag = 1
    return distance, flag


def move_wheels(v_left, v_right):
    """ move the wheels. Input: Angular velocities in rad/s """
    global clientID, left_motorID, right_motorID
    vrep.simxSetJointTargetVelocity(clientID, left_motorID, v_left, STREAMING)
    vrep.simxSetJointTargetVelocity(clientID, right_motorID, v_right, STREAMING)
    time.sleep(config.SLEEP_TIME * 2)


def get_reward_distance():
    """ return the reference distance for reward """
    global reward_ref, clientID
    res, reward_ref = vrep.simxReadDistance(clientID, rewardRefID, BUFFER)
    return reward_ref


def stop_motion():
    """ stop the base wheels """
    global clientID, left_motorID, right_motorID
    vrep.simxSetJointTargetVelocity(clientID, left_motorID, 0, STREAMING)
    vrep.simxSetJointTargetVelocity(clientID, right_motorID, 0, STREAMING)
    time.sleep(config.SLEEP_TIME * 3)


def is_collided_with_wall():
    """ judge if collision happens"""
    global clientID, wall0_collisionID, wall1_collisionID, wall2_collisionID, wall3_collisionID
    for id in [wall0_collisionID, wall1_collisionID, wall2_collisionID, wall3_collisionID]:
        res, status = vrep.simxReadCollision(clientID, id, BUFFER)
        if status:
            print("Collision has been detected!")
            return True
    return False


def is_collided_with_target():
    global clientID, target_collisionID
    res, flag = vrep.simxReadCollision(clientID, target_collisionID, BUFFER)
    return flag

