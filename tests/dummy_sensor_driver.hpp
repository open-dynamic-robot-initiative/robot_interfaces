/**
 * @file
 * @brief Dummy sensor driver for testing.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <chrono>
#include <thread>

#include <robot_interfaces/sensors/sensor_driver.hpp>

namespace robot_interfaces
{
namespace testing
{
/**
 * @brief Simple dummy sensor driver for testing.
 *
 * This driver uses `int` as observation type and returns as observation a
 * sequence of increasing numbers, starting at zero.
 */
class DummySensorDriver : public robot_interfaces::SensorDriver<int>
{
public:
    int get_observation() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return counter++;
    }

private:
    int counter = 0;
};
}  // namespace testing
}  // namespace robot_interfaces
