/**
 * @file
 * @brief Base driver for the sensors.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <iostream>

#include <robot_interfaces/utils.hpp>

namespace robot_interfaces
{
/**
 * @brief Base driver class from which all specific sensor
 * drivers should derive.
 *
 * @tparam ObservationType
 */
template <typename ObservationType, typename InfoType = None>
class SensorDriver
{
public:
    // virtual destructor is needed for class with virtual methods
    virtual ~SensorDriver()
    {
    }

    /**
     * @brief Return static information about the sensor.
     *
     * This information is expected to be constructed during initialization and
     * to not change later on.
     */
    virtual InfoType get_sensor_info()
    {
        return InfoType();
    }

    /**
     * @brief return the observation
     * @return depends on the observation structure
     * of the sensor being interacted with
     */
    virtual ObservationType get_observation() = 0;
};
}  // namespace robot_interfaces
