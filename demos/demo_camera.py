#!/usr/bin/env python3
"""
This demo is to start a camera and display the images from it
as a non-real time livestream.

Basically illustrates what objects to create to interact with the
camera, and the available methods for that.
"""
import numpy as np
import cv2

import robot_interfaces


def main():
    camera_data = robot_interfaces.camera.SensorData()
    camera_driver = robot_interfaces.camera.OpenCVDriver()
    camera_backend = robot_interfaces.camera.SensorBackend(
                                        camera_driver, camera_data)
    camera_frontend = robot_interfaces.camera.SensorFrontend(camera_data)

    while True:
        observation = camera_frontend.get_latest_observation()
        window_name = "Image Stream"
        cv2.imshow(window_name, np.array(observation.image))
        print("type", type(observation.image))
        cv2.waitKey(3)


if __name__ == "__main__":
    main()
