#pragma once

#include <ctime>
#include <iostream>

#include <real_time_tools/timer.hpp>
#include <opencv2/opencv.hpp>

namespace robot_interfaces
{
/**
 * @brief Driver for interacting with any camera using OpenCV.
 */
template <typename OpenCVObservation>
class OpenCVDriver
{
public:
    cv::VideoCapture video_capture_;
    float start_time_ = 0;

    OpenCVDriver()
    {
        cv::VideoCapture cap(start_time_);
        video_capture_ = cap;
    }

    /**
     * @brief Find out if the camera can be accessed and
     * if the video capture has been started.
     */
    bool is_grabbing_successful()
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
    OpenCVObservation grab_frame()
    {
        OpenCVObservation image_frame;
        cv::Mat frame;
        long double current_time = real_time_tools::Timer::get_current_time_sec();
        video_capture_ >> frame;
        image_frame.image = frame;
        image_frame.time_stamp = current_time;
        return image_frame;
    }
};

}  // namespace robot_interfaces