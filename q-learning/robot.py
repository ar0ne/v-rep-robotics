#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import config
import vrepInterface
import os
import pickle
import threading
import qlearning
from collections import namedtuple

states = namedtuple('states', 'l r')


class LearningAgent(object):
    def __init__(self, restore):
        import sys
        if sys.version_info[0] < 3:
            sys.exit("Sorry, Python 3 required")

        self.state = None
        self.action = None
        self.reward = 0.0
        self.next_state = None
        self.next_action = None

        self.done = False
        self.t = 0
        self.flag = 0
        self.flag_prev = 0
        self.deadline = config.DEADLINE
        self.num_out_of_time = 0
        self.hit_wall_time = 0
        self.ok_time = 0
        self.far_time = 0
        self.ai = qlearning.RL()
        self.epi = 0

        parent_path = os.path.dirname(os.path.realpath(__file__))
        data_path = os.path.join(parent_path, config.DB_FOLDER)
        if not os.path.exists(data_path):
            os.makedirs(data_path)
        self.data = os.path.join(data_path, config.DB_NAME)

        if restore:
            if os.path.exists(self.data):
                print('restoring trained data from {}'.format(data_path))
                with open(self.data, 'rb') as f:
                    self.ai.q = pickle.load(f)
                    print('trainedData length:', len(self.ai.q))
                    print('restoring done ...')

    def run(self):
        vrepInterface.connect()
        train_thread = threading.Thread(name="train", target=self.train())
        train_thread.daemon = True
        train_thread.start()
        while True:
            continue

    def train(self):
        train_number = 1
        while self.ok_time < config.EPOCHS:
            print("Train #{}".format(train_number))
            vrepInterface.start()
            self.reset()
            self.done = False
            self.t = 0

            while True:
                quit_flag = False
                try:
                    self.step()
                except KeyboardInterrupt:
                    quit_flag = True
                finally:
                    if quit_flag or self.done:
                        break

            if train_number % 10:
                self.save_progress(train_number)

            self.show_statistics(train_number)

            train_number += 1

            # Update the learning rate
            self.ai.alpha = pow(train_number, -0.1)

    def show_statistics(self, epi):
        print("Success: {} / {}".format(self.ok_time, epi))
        print("Collision: {} / {}".format(self.hit_wall_time, epi))
        print("too far: {} / {}".format(self.far_time, epi))
        print("Out of time: {} / {}".format(self.num_out_of_time, epi))

    def save_progress(self, epi):
        print("Train {} done, saving Q table...".format(epi))
        with open(self.data, 'wb') as f:
            pickle.dump(self.ai.q, f, protocol=pickle.HIGHEST_PROTOCOL)

    def reset(self):
        self.state = None
        self.action = None
        self.reward = None

    def step(self):

        self.update()

        if self.done:
            return

        if self.t >= self.deadline:
            print("Agent ran out of time limit!")
            self.done = True
            self.num_out_of_time += 1

        self.t += 1

    def update(self):

        if self.state is None:
            s, self.flag = vrepInterface.get_ultra_distance()
            self.state = states(l=s[0], r=s[1])
            self.action = self.ai.choose_next_action(self.state)

        self.next_state = self.get_next_state()
        self.reward = self.get_reward()

        self.next_action = self.ai.choose_next_action(self.next_state)

        updated_q = self.ai.get_updated_q(self.state, self.action, self.reward, self.next_state)

        self.ai.q[(self.state, self.action)] = updated_q

        print("step = {}, state = {}, action = {}, next_action = {}, reward = {}".format(self.t, self.state,
                                                                                         self.action, self.next_action,
                                                                                         self.reward))

        print("q_cur = {0:.3f}".format(self.ai.getQ(self.state,
                                                    self.action)), end=", ")

        print("q_next = {0:.3f}".format(self.ai.getQ(self.state,
                                                     self.next_action)))

        self.state = self.next_state
        self.action = self.next_action

    def get_next_state(self):
        speed_left, speed_right = config.valid_actions_dict[self.action]

        vrepInterface.move_wheels(speed_left, speed_right)
        vrepInterface.stop_motion()

        self.flag_prev = self.flag
        n_s, self.flag = vrepInterface.get_ultra_distance()

        return states(l=n_s[0], r=n_s[1])

    def get_reward(self):
        if vrepInterface.is_collided_with_wall():
            self.hit_wall_time += 1
            self.done = True
            return -100
        reward_distance = vrepInterface.get_reward_distance()
        if reward_distance < config.tolerance or vrepInterface.is_collided_with_target():
            print("Success!")
            self.done = True
            self.ok_time += 1
            return 100
        elif reward_distance >= config.MAX_DISTANCE or self.state == states(l=-1.0, r=-1.0):
            print("too far!")
            self.done = True
            self.far_time += 1
            return -100
        else:
            # return -1.
            # return 1. / reward_ref
            return ( config.MAX_DISTANCE - reward_distance ) / config.MAX_DISTANCE


if __name__ == '__main__':
    agent = LearningAgent(restore=config.restore)
    agent.run()
