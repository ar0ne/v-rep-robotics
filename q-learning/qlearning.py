#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import random
import config
from itertools import product

class RL:
    def __init__(self, actions=config.valid_actions, epsilon=config.epsilon, alpha=config.alpha, gamma=config.gamma):
        self.q = {}

        self.actions = actions
        self.epsilon = epsilon  # exploration constant of epsilon-greedy algorithm
        self.alpha = alpha  # discount constant
        self.gamma = gamma  # learning rate

        self.randomize = False

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def choose_next_action(self, state):  # get action using epsilon-greedy algorithm
        if self.randomize and random.random() < self.epsilon:
            return random.choice(self.actions)
        else:
            possible_states = self.get_neighbours(state)
            q = []
            for act in self.actions:
                vs = []
                for st in possible_states:
                    value = self.getQ(st, act)
                    vs.append(value)
                max_v = max(vs)
                q.append((act, max_v))

            max_q_v = max([q[i][1] for i in range(len(q))])
            count = [maxV[1] for maxV in q].count(max_q_v)
            if count > 1:
                return random.choice([q[i][0] for i in range(len(q)) if q[i][1] == max_q_v])
            return [q[i][0] for i in range(len(q)) if max_q_v == q[i][1]][0]

    def choose_next_action_standart(self, state):
        if self.randomize and random.random() < self.epsilon:
            return random.choice(self.actions)
        else:
            q = [self.getQ(state, a) for a in self.actions]
            maxQ = max(q)
            count = q.count(maxQ)
            if count > 1:
                best = [i for i in range(len(self.actions)) if q[i] == maxQ]
                return self.actions[random.choice(best)]
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

    def get_neighbours(self, state):
        steps = [x * 0.01 for x in range(-1, 2)]
        l_states = [state.l + st for st in steps]
        r_states = [state.r + st for st in steps]
        res = []
        for pair in product(l_states, r_states):
            res.append(pair)
        return res