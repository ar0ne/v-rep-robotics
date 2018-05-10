#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np

EPOCHS = 500
restore = True
alpha = 0.9
gamma = 0.8
epsilon = 0.3
valid_actions = ['forward', 'left_forward', 'right_forward']

SPEED = 3
SLEEP_TIME = 0.1

wait_response = False
ultra_distribution = ['left_ultra', 'right_ultra']
n_ultra = len(ultra_distribution)
valid_actions_dict = {valid_actions[0]: np.array([SPEED, SPEED]),
                      valid_actions[1]: np.array([-SPEED, SPEED]),
                      valid_actions[2]: np.array([SPEED, -SPEED])}
tolerance = 0.05
grid_width = 0.002

MAX_DISTANCE = 1.4

DEADLINE = 100

DB_NAME = "data.pkl"
DB_FOLDER = "save"

PRECISION = 2

RANDOM_ACTIIVE = False
