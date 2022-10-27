
/**
 * @file
 * @brief To store all the data from all the sensors in use
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <time_series/multiprocess_time_series.hpp>
#include <time_series/time_series.hpp>

namespace robot_interfaces
{
/**
 * @brief Contains the data coming from the sensors.
 *
 * @tparam Observation  Type of the sensor observation.
 */
template <typename Observation>
class SensorData
{
public:
    typedef std::shared_ptr<SensorData<Observation>> Ptr;
    typedef std::shared_ptr<const SensorData<Observation>> ConstPtr;

    //! @brief Time series of the sensor observations.
    std::shared_ptr<time_series::TimeSeriesInterface<Observation>> observation;

protected:
    // make constructor protected to prevent instantiation of the base class
    SensorData(){};
};

/**
 * @brief SensorData instance using single process time series.
 *
 * Use this class if all modules accessing the data are running in the same
 * process.  If modules run in separate processes, use MultiProcessSensorData
 * instead.
 *
 * @copydoc SensorData
 * @see MultiProcessSensorData
 */
template <typename Observation>
class SingleProcessSensorData : public SensorData<Observation>
{
public:
    SingleProcessSensorData(size_t history_length = 1000)
    {
        this->observation =
            std::make_shared<time_series::TimeSeries<Observation>>(
                history_length);
    }
};

/**
 * @brief SensorData instance using multi process time series.
 *
 * Use this class if modules accessing the data are running in separate
 * processes.  When all modules run as threads in the same process, this class
 * can be used as well, however, SingleProcessSensorData might be more efficient
 * in that case.
 *
 * @copydoc SensorData
 * @see SingleProcessSensorData
 */
template <typename Observation>
class MultiProcessSensorData : public SensorData<Observation>
{
public:
    MultiProcessSensorData(const std::string &shared_memory_id,
                           bool is_master,
                           size_t history_length = 1000)
    {
        if (is_master)
        {
            // the master instance is in charge of cleaning the memory
            time_series::clear_memory(shared_memory_id);

            this->observation = time_series::MultiprocessTimeSeries<
                Observation>::create_leader_ptr(shared_memory_id,
                                                history_length);
        }
        else
        {
            this->observation = time_series::MultiprocessTimeSeries<
                Observation>::create_follower_ptr(shared_memory_id);
        }
    }
};

}  // namespace robot_interfaces
