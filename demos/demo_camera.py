#!/usr/bin/env python3

import time
import numpy as np
import pickle
import cv2

import robot_interfaces

camera_data = robot_interfaces.camera.SensorData()
camera_handle = robot_interfaces.camera.OpenCVDriver()
camera_backend = robot_interfaces.camera.SensorBackend(camera_handle, camera_data)
camera_frontend = robot_interfaces.camera.SensorFrontend(camera_data)

CameraObservation = robot_interfaces.camera.OpenCVObservation

num_imgs = 100000000
img_list = []

for i in range(num_imgs):
    OpenCVObservation = (camera_frontend.get_latest_observation())
    img_list.append(OpenCVObservation)
    print("At image", i)
    window_name = 'image'
    cv2.imshow(window_name, np.array(OpenCVObservation.image))
    cv2.waitKey(3)

pickle.dump(img_list, open("saved_imgs.p", "wb")) 