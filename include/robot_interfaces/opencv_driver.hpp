/**
 * @file
 * @brief Driver to interface with the camera using OpenCV.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <ctime>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <real_time_tools/timer.hpp>
#include <robot_interfaces/camera_observation.hpp>
#include <robot_interfaces/sensor_driver.hpp>

namespace robot_interfaces
{
/**
 * @brief Driver for interacting with any camera using OpenCV.
 */
class OpenCVDriver : public SensorDriver<CameraObservation>
{
public:
    cv::VideoCapture video_capture_;
    // double device_id_;

    OpenCVDriver(int device_id)
    {
        cv::VideoCapture cap(device_id);
        video_capture_ = cap;
    }

    /**
     * @brief Find out if the camera can be accessed and
     * if the video capture has been started.
     */

    bool is_access_successful()
    {
        if (!video_capture_.isOpened())
        {
#ifdef VERBOSE
            std::cout << "Could not access camera stream :(" << std::endl;
#endif
            return false;
        }
        else
        {
#ifdef VERBOSE
            std::cout << "Succeeded in accessing camera stream!" << std::endl;
#endif
            return true;
        }
    }

    /**
     * @brief Grab a single frame along with its timestamp.
     *
     * @return Image frame consisting of an image matrix and the time at
     * which it was grabbed.
     */

    CameraObservation get_observation()
    {
        CameraObservation image_frame;
        cv::Mat frame;
        double current_time = real_time_tools::Timer::get_current_time_sec();
        video_capture_ >> frame;
        image_frame.image = frame;
        image_frame.time_stamp = current_time;
        return image_frame;
    }
};

}  // namespace robot_interfaces