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

#include <robot_interfaces/utils.hpp>

namespace robot_interfaces
{
/**
 * @brief Contains the data coming from the sensors.
 *
 * @tparam Observation  Type of the sensor observation.
 */
template <typename Observation, typename Info = None>
class SensorData
{
public:
    typedef std::shared_ptr<SensorData<Observation, Info>> Ptr;
    typedef std::shared_ptr<const SensorData<Observation, Info>> ConstPtr;

    /**
     * @brief Static information about the sensor
     *
     * Note: A time series is used here for convenience to handle the shared
     * memory aspect.  However, this is intended to only hold one element that
     * doesn't change over time.
     */
    std::shared_ptr<time_series::TimeSeriesInterface<Info>> sensor_info;

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
template <typename Observation, typename Info = None>
class SingleProcessSensorData : public SensorData<Observation, Info>
{
public:
    SingleProcessSensorData(size_t history_length = 1000)
    {
        // sensor_info only contains a single static element, so length is set
        // to 1
        this->sensor_info = std::make_shared<time_series::TimeSeries<Info>>(1);
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
template <typename Observation, typename Info = None>
class MultiProcessSensorData : public SensorData<Observation, Info>
{
public:
    MultiProcessSensorData(const std::string &shared_memory_id,
                           bool is_master,
                           size_t history_length = 1000)
    {
        // each time series needs its own shared memory ID, so add unique
        // suffixes to the given ID.
        const std::string shm_id_info = shared_memory_id + "_info";
        const std::string shm_id_observation =
            shared_memory_id + "_observation";

        if (is_master)
        {
            // the master instance is in charge of cleaning the memory
            time_series::clear_memory(shm_id_info);
            time_series::clear_memory(shm_id_observation);

            // sensor_info only contains a single static element, so length is
            // set to 1
            this->sensor_info =
                time_series::MultiprocessTimeSeries<Info>::create_leader_ptr(
                    shm_id_info, 1);

            this->observation = time_series::MultiprocessTimeSeries<
                Observation>::create_leader_ptr(shm_id_observation,
                                                history_length);
        }
        else
        {
            this->sensor_info =
                time_series::MultiprocessTimeSeries<Info>::create_follower_ptr(
                    shm_id_info);

            this->observation = time_series::MultiprocessTimeSeries<
                Observation>::create_follower_ptr(shm_id_observation);
        }
    }
};

}  // namespace robot_interfaces
