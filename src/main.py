#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'


from bug import *

import argparse
import sys

def create_parser():
    parser = argparse.ArgumentParser(prog="main.py",
                                     description='''Implementation of Bug's algorithms for V-REP simulator''',
                                     epilog=''' Serj Ar[]ne 2015 ''')

    parser.add_argument('-a', '--algorithm', choices=['BUG1', 'BUG2', 'DISTBUG'], help="Choose bug algorithm.", default='DISTBUG')
    parser.add_argument('-s', '--speed', type=int, help="Speed of roobot wheels", default=1.0)
    parser.add_argument('-t', '--targetname', type=str, help="Name of target on scene", default='target')
    parser.add_argument('-b', '--botname', type=str, help="Name of bot on scene", default='Bot')

    return parser


if __name__ == '__main__':

    parser = create_parser()
    namespace = parser.parse_args(sys.argv[1:])

    bug = None

    if namespace.algorithm == "BUG1":
        bug = Bug1(target_name=namespace.targetname, bot_name=namespace.botname, wheel_speed=namespace.speed)
    elif namespace.algorithm == "BUG2":
        bug = Bug2(target_name=namespace.targetname, bot_name=namespace.botname, wheel_speed=namespace.speed)
    elif namespace.algorithm == "DISTBUG":
        bug = DistBug(target_name=namespace.targetname, bot_name=namespace.botname, wheel_speed=namespace.speed)
    else:
        print("Something goes wrong!")
        exit(-2)

    bug.loop()

