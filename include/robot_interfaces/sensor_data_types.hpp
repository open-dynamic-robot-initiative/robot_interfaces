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

struct SensorDataTypes
{
    typedef cv::Mat Image;
    typedef long double TimeStamp;

    /**
     * @brief Observation structure to store cv::Mat images with corresponding
     * timestamps.
     */
    struct OpenCVObservation
    {
        Image image;
        TimeStamp time_stamp;
    };

    typedef SensorData<OpenCVObservation> Data;
    typedef std::shared_ptr<Data> DataPtr;
    typedef PylonDriver<OpenCVObservation> CVDriver;
    typedef std::shared_ptr<CVDriver> CVDriverPtr;
    typedef SensorFrontend<OpenCVObservation> Frontend;
    typedef SensorBackend<OpenCVObservation> Backend;
};

}  // namespace robot_interfaces
