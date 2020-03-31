#!/usr/bin/env python3
"""
This demo is to start three pylon dependent cameras and store a list
of 100 observations obtained from them as a pickle file.
"""
import numpy as np
import cv2
import pickle

import robot_interfaces


def main():

    camera_data = robot_interfaces.tricamera.Data()
    camera_driver = robot_interfaces.tricamera.SyncDriver("cam_1", "cam_2", "cam_3")


    camera_backend = robot_interfaces.tricamera.Backend(
                                        camera_driver, camera_data)
    camera_frontend = robot_interfaces.tricamera.Frontend(camera_data)
    img_list = []

    for _ in range(100):
        observation = camera_frontend.get_latest_observation()
        img_list.append(observation)

    pickle.dump(img_list, open("img_list.p", "wb"))


if __name__ == "__main__":
    main()
