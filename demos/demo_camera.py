#!/usr/bin/env python3

import time
import numpy as np
import pickle

import robot_interfaces

camera_data = robot_interfaces.camera.SensorData()
print("Check1")
camera_handle = robot_interfaces.camera.OpenCVDriver()
print("Check2")
camera_backend = robot_interfaces.camera.SensorBackend(camera_handle, camera_data)
print("Check3")
camera_frontend = robot_interfaces.camera.SensorFrontend(camera_data)
print("Check4")

num_imgs = 10
img_list = []

for i in range(num_imgs):
    # print("Check5")
    # img = camera_frontend.get_observation()
    # print("Check6")
    # img_list.append(img)
    # print("At image", i)
    print("The current time index is", camera_frontend.get_current_timeindex())

# pickle.dump(img_list, open("saved_imgs.p", "wb")) 