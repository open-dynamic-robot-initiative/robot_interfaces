#!/usr/bin/env python3
"""
This demo is to start three pylon dependent cameras, view their image streams,
and if desired, store the timestamps from them to analyse how well the three
cameras are in sync with each other.
"""
import argparse
import numpy as np
import cv2
import pickle

import robot_interfaces


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    arg_action_group = argparser.add_mutually_exclusive_group(required=False)
    arg_action_group.add_argument(
        "--store_timestamps",
        type=str,
        help="""Pass --store_timestamps to dump the timestamps from the observations
             of the three cameras into a pickle file, followed by the name of
             this pickle file (for eg. time_stamps.p).
             """,
    )
    args = argparser.parse_args()

    camera_data = robot_interfaces.tricamera.Data()
    camera_driver = robot_interfaces.tricamera.TriCameraDriver(
        "camera60", "camera180", "camera300"
    )

    camera_backend = robot_interfaces.tricamera.Backend(
        camera_driver, camera_data
    )
    camera_frontend = robot_interfaces.tricamera.Frontend(camera_data)
    observations_timestamps_list = []

    for _ in range(100):
        observation = camera_frontend.get_latest_observation()
        window_60 = "Image Stream camera60"
        window_180 = "Image Stream camera180"
        window_300 = "Image Stream camera300"
        cv2.imshow(
            window_180, np.array(observation.cameras[0].image, copy=False)
        )
        cv2.imshow(
            window_300, np.array(observation.cameras[1].image, copy=False)
        )
        cv2.imshow(
            window_60, np.array(observation.cameras[2].image, copy=False)
        )
        cv2.waitKey(3)

        observations_timestamps_list.append(
            [
                observation.cameras[0].time_stamp,
                observation.cameras[1].time_stamp,
                observation.cameras[2].time_stamp,
            ]
        )

    if args.store_timestamps:
        pickle.dump(observations_timestamps_list, open("time_stamps.p", "wb"))


if __name__ == "__main__":
    main()
