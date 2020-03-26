import numpy as np
import cv2

import robot_interfaces


def main():

    camera_data = robot_interfaces.tricamera.Data()
    camera_driver = robot_interfaces.tricamera.SyncDriver("cam_1", "cam_2", "cam_3")


    camera_backend = robot_interfaces.tricamera.Backend(
                                        camera_driver, camera_data)
    camera_frontend = robot_interfaces.tricamera.Frontend(camera_data)

    while True:
        observation = camera_frontend.get_latest_observation()
        # window_name = "Image Stream"
        # cv2.imshow(window_name, np.array(observation.image, copy=False))
        # cv2.waitKey(3)

if __name__ == "__main__":
    main()