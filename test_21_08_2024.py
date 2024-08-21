#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Import library for project
"""


from __future__ import print_function
from dronekit import VehicleMode, LocationGlobalRelative

import subprocess

import time

from flyUnit import FlyUnit
from camera import Camera
from control import *
from socket_io import *
from stream import RTSPStreamer
from sensor import *
from qr import *
import threading

"""
Defined variable for project
"""
DELAY_SEND_INFO = .6
DELAY_ACTION = .8

time_prev_send_info = 0
time_prev_action = 0

time_curr = 0


"""
Class Drone to handle event of drone to project
"""

class Drone:
    def __init__(self) -> None:
        """
            _init_
        """
        self.copter = FlyUnit()
    

def main():
    """
        _main_
    """
    drone = Drone()
    
    if drone.copter.isConnect:
        print("drone is connected")

if __name__ == __name__:
    """
        _start_
    """
    print("__start__")

    main()

