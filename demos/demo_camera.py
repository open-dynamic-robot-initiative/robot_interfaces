#!/usr/bin/env python3
"""Send zero-torque commands to the robot and create a log"""

import time
import numpy as np

import robot_interfaces

camera_data = robot_interfaces.camera.CData()
camera_backend = robot_interfaces.camera.CBackend(camera_data)
camera_frontend = robot_interfaces.camera.CFrontend(camera_data)
camera_handle = robot_interfaces.camera.CDriver()

num_imgs = 100
img_list = []

for i in range(num_imgs):
    img = camera_handle.grab_frame()
    img_list.append(img)
