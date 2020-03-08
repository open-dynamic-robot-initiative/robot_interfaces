#!/usr/bin/env python3

import time
import numpy as np
import pickle
# import cv2

import robot_interfaces

camera_data = robot_interfaces.camera.SensorData()
print("Check1")
camera_handle = robot_interfaces.camera.OpenCVDriver()
print("Check2")
print("has grabbing started?", camera_handle.is_grabbing_successful())
camera_backend = robot_interfaces.camera.SensorBackend(camera_handle, camera_data)
print("Check3")
camera_frontend = robot_interfaces.camera.SensorFrontend(camera_data)
print("Check4")

CameraObservation = robot_interfaces.camera.CameraObservation

num_imgs = 10
img_list = []

for i in range(num_imgs):
    print("Check5")
    # print("To store in type", (camera_handle.grab_frame()).image.type)
    CameraObservation = (camera_frontend.get_latest_observation())
    print("Check6", CameraObservation)
    img_list.append(CameraObservation)
    print("At image", i)
    # print("camera data", camera_data)
    # print("The current time index is", camera_frontend.get_current_timeindex())

pickle.dump(img_list, open("saved_imgs.p", "wb")) 