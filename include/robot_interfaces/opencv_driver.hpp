#pragma once

#include <ctime>
#include <iostream>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>
#include "opencv2/opencv.hpp"

namespace robot_interfaces
{
/**
 * @brief Driver for interacting with any camera using OpenCV.
 */

template <typename OpenCVObservation>
class OpenCVDriver
{
public:
    cv::VideoCapture video_capture;
    real_time_tools::Timer timer;

    OpenCVDriver()
    {
        cv::VideoCapture cap(0);
        video_capture = cap;
    }

    /**
     * @brief Find out if the camera can be accessed and
     * if the video capture has been started.
     */

    int is_grabbing_successful()
    {
        if (!video_capture.isOpened())
        {
#ifdef VERBOSE
            std::cout << "Could not access camera stream :(" << std::endl;
#endif
            return -1;
        }
        else
        {
#ifdef VERBOSE
            std::cout << "Succeeded in accessing camera stream!" << std::endl;
#endif
            return 1;
        }
    }

    /**
     * @brief Grab frames along with their timestamps one by one.
     *
     * @return Image frame consisting of an image matrix and the time at
     * which it was grabbed.
     */

    OpenCVObservation grab_frame()
    {
        OpenCVObservation image_frame;
        cv::Mat frame;
        long double current_time = timer.get_current_time_sec();
        video_capture >> frame;
        image_frame.image = frame;
        image_frame.time_stamp = current_time;
        return image_frame;
    }
};

}  // namespace robot_interfaces