/**
 * @file
 * @brief Defines data structures for each of the different sensors.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <cmath>
#include <ctime>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <time_series/time_series.hpp>

#include <opencv2/opencv.hpp>

#include <robot_interfaces/opencv_driver.hpp>
#include <robot_interfaces/sensor_backend.hpp>
#include <robot_interfaces/sensor_data.hpp>
#include <robot_interfaces/sensor_frontend.hpp>

namespace robot_interfaces
{
/**
 * @brief Collection of data types for the sensors used in the trifinger
 * platform.
 *
 * Refer to the docstrings of individual data types for more details.
 */

struct CameraObservation
{
    typedef cv::Mat Image;
    typedef double TimeStamp;
    Image image;
    TimeStamp time_stamp;

    /**
     * @brief Observation structure to store cv::Mat images with corresponding
     * timestamps.
     */
    // struct CameraObservation
    // {
    //     typedef cv::Mat Image;
    //     typedef double TimeStamp;
    //     Image image;
    //     TimeStamp time_stamp;
    // };

    typedef SensorData<CameraObservation> Data;
    typedef std::shared_ptr<Data> DataPtr;
    typedef OpenCVDriver<CameraObservation> CVDriver;
    typedef std::shared_ptr<CVDriver> CVDriverPtr;
    typedef SensorFrontend<CameraObservation> Frontend;
    typedef SensorBackend<CameraObservation> Backend;
};

}  // namespace robot_interfaces
