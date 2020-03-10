#!/usr/bin/env python3

import time
import numpy as np
import cv2

import robot_interfaces

camera_data = robot_interfaces.camera.SensorData()
camera_driver = robot_interfaces.camera.OpenCVDriver()
camera_backend = robot_interfaces.camera.SensorBackend(
                                        camera_driver, camera_data)
camera_frontend = robot_interfaces.camera.SensorFrontend(camera_data)

while True:
    observation = camera_frontend.get_latest_observation()
    window_name = "Image Stream"
    cv2.imshow(window_name, np.array(observation.image))
    cv2.waitKey(3)
