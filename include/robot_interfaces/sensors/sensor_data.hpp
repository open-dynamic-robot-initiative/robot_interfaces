
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

#include <time_series/time_series.hpp>

namespace robot_interfaces
{
template <typename Type>
using Timeseries = time_series::TimeSeries<Type>;

/**
 * @brief Contains all the data coming from the sensors.
 *
 * @tparam ObservationType
 */

template <typename ObservationType>
class SensorData
{
public:
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;

    SensorData(size_t history_length = 15000,
               bool use_shared_memory = false,
               // suppress unused warning (will be used in the future)
               __attribute__((unused)) std::string shared_memory_address = "")
    {
        if (use_shared_memory)
        {
            std::cout << "shared memory sensor data is not implemented yet"
                      << std::endl;
            exit(-1);

            // TODO: here we should check if the shared memory at that
            // address already exists, otherwise we create it.
            // we will also have to update timeseries such as to handle
            // serialization internally (it will simply assume that the
            // templated class has a method called serialize() and
            // from_serialized())
        }
        else
        {
            observation =
                std::make_shared<Timeseries<ObservationType>>(history_length);
        }
    }

public:
    Ptr<Timeseries<ObservationType>> observation;
};

}  // namespace robot_interfaces
