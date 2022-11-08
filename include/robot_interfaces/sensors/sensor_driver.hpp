/**
 * @file
 * @brief Base driver for the sensors.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <iostream>

namespace robot_interfaces
{
/**
 * @brief Base driver class from which all specific sensor
 * drivers should derive.
 *
 * @tparam ObservationType
 */
template <typename ObservationType>
class SensorDriver
{
public:
    // virtual destructor is needed for class with virtual methods
    virtual ~SensorDriver()
    {
    }

    /**
     * @brief return the observation
     * @return depends on the observation structure
     * of the sensor being interacted with
     */
    virtual ObservationType get_observation() = 0;
};
}  // namespace robot_interfaces
