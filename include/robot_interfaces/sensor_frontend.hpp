#pragma once

#include <algorithm>
#include <cmath>

#include <time_series/time_series.hpp>

#include <robot_interfaces/sensor_data.hpp>

namespace robot_interfaces
{

/**
 * @brief Communication link between SensorData and the user.
 *
 * Exposes the sensor data to the user via hardware-friendly methods.
 *
 * @tparam OpenCVObservation
 */
template <typename OpenCVObservation>
class SensorFrontend
{
public:
    template <typename Type>
    using Timeseries = time_series::TimeSeries<Type>;
    typedef time_series::Timestamp TimeStamp;
    typedef time_series::Index TimeIndex;

    SensorFrontend(std::shared_ptr<SensorData<OpenCVObservation>> sensor_data)
        : sensor_data_(sensor_data)
    {
    }

    OpenCVObservation get_observation(const TimeIndex &t)
    {
        return (*sensor_data_->observation)[t];
    }

    OpenCVObservation get_latest_observation()
    {
        return sensor_data_->observation->newest_element();
    }

    TimeStamp get_timestamp_ms(const TimeIndex &t)
    {
        return sensor_data_->observation->timestamp_ms(t);
    }
    TimeIndex get_current_timeindex()
    {
        return sensor_data_->observation->newest_timeindex();
    }

private:
    std::shared_ptr<SensorData<OpenCVObservation>> sensor_data_;
};

}  // namespace robot_interfaces
