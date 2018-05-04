#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import random
import config


class RL:
    def __init__(self, actions=config.valid_actions, epsilon=config.epsilon, alpha=config.alpha, gamma=config.gamma):
        self.q = {}

        self.actions = actions
        self.epsilon = epsilon  # exploration constant of epsilon-greedy algorithm
        self.alpha = alpha  # discount constant
        self.gamma = gamma  # learning rate

        self.randomly = True

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def choose_next_action(self, state):  # get action using epsilon-greedy algorithm
        if self.randomly and random.random() < self.epsilon:
            return random.choice(self.actions)
        else:
            # TODO: add neighbour states
            q = [self.getQ(state, a) for a in self.actions]
            maxQ = max(q)
            count = q.count(maxQ)
            # in case there're several state-action max values, we select a random one among them
            if count > 1:
                best = [i for i in range(len(self.actions)) if q[i] == maxQ]
                return self.actions[random.choice(best)]
            else:
                return self.actions[q.index(maxQ)]

    # after choosing action, the agent needs to interact with the environment to get reward and the next state
    # then it can update Q value
    def get_updated_q(self, last_state, last_action, reward, new_state):  # update Q value according to reward and new state
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + gamma * maxQ(s') - Q(s,a))
        '''
        oldq = self.getQ(last_state, last_action)

        return oldq + self.alpha * (reward + self.gamma * self.get_max_q(new_state) - oldq)

    def get_max_q(self, next_state):
        """get the Q value of s' according to greedy algorithm."""
        return max([self.getQ(next_state, a) for a in self.actions])
