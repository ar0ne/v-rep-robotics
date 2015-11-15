import vrep
import sys
import pygame

pygame.init()
pygame.display.set_mode((200, 100))

vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print("Connected to remote server")
else:
    print('Connection not successful')
    sys.exit('Could not connect')

errorCode, left_motor_handle  = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',  vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

if errorCode == -1:
    print('Can not find left or right motor')
    sys.exit()

SPEED_MAX = 0.2  
speed_left = 0.
speed_right = 0.              

pressed_left = pressed_right = pressed_up = pressed_down = False

while True:

    for event in pygame.event.get():
        if event.type == pygame.QUIT: 
            sys.exit()        
        elif event.type == pygame.KEYDOWN:          # check for key presses          
            if event.key == pygame.K_LEFT:        # left arrow turns left
                pressed_left = True
            elif event.key == pygame.K_RIGHT:     # right arrow turns right
                pressed_right = True
            elif event.key == pygame.K_UP:        # up arrow goes up
                pressed_up = True
            elif event.key == pygame.K_DOWN:     # down arrow goes down
                pressed_down = True
        elif event.type == pygame.KEYUP:            # check for key releases
            if event.key == pygame.K_LEFT:        # left arrow turns left
                pressed_left = False
            elif event.key == pygame.K_RIGHT:     # right arrow turns right
                pressed_right = False
            elif event.key == pygame.K_UP:        # up arrow goes up
                pressed_up = False
            elif event.key == pygame.K_DOWN:     # down arrow goes down
                pressed_down = False

    if pressed_left:
        speed_right += SPEED_MAX
    elif speed_right != 0:
        

    if pressed_right:
        speed_left  += SPEED_MAX
    if pressed_up:
        speed_right += SPEED_MAX
        speed_left  += SPEED_MAX
    if pressed_down:
        speed_right += - SPEED_MAX
        speed_left  += - SPEED_MAX

    vrep.simxSetJointTargetVelocity(clientID, left_motor_handle,  speed_left,  vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, speed_right, vrep.simx_opmode_oneshot_wait)
