/**
 * @file
 * @brief Connects the driver with sensor data
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <thread>

#include <robot_interfaces/sensors/sensor_data.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>

namespace robot_interfaces
{
/**
 * @brief Communication link between SensorData and SensorDriver.
 *
 * At each instant, it checks if the sensor can be accessed, and
 * then gets the observation from it (the observation type depends
 * on the sensor) and appends it to the sensor data.
 *
 * @tparam ObservationType
 */
template <typename ObservationType>
class SensorBackend
{
public:
    typedef std::shared_ptr<SensorBackend<ObservationType>> Ptr;
    typedef std::shared_ptr<const SensorBackend<ObservationType>> ConstPtr;

    /**
     * @param sensor_driver  Driver instance for the sensor.
     * @param sensor_data  Data is sent to/retrieved from here.
     */
    SensorBackend(std::shared_ptr<SensorDriver<ObservationType>> sensor_driver,
                  std::shared_ptr<SensorData<ObservationType>> sensor_data)
        : sensor_driver_(sensor_driver),
          sensor_data_(sensor_data),
          shutdown_requested_(false)
    {
        thread_ = std::thread(&SensorBackend<ObservationType>::loop, this);
    }

    // reinstate the implicit move constructor
    // See https://stackoverflow.com/a/27474070
    SensorBackend(SensorBackend &&) = default;

    //! @brief Stop the backend thread.
    void shutdown()
    {
        shutdown_requested_ = true;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    virtual ~SensorBackend()
    {
        shutdown();
    }

private:
    std::shared_ptr<SensorDriver<ObservationType>> sensor_driver_;
    std::shared_ptr<SensorData<ObservationType>> sensor_data_;

    bool shutdown_requested_;

    std::thread thread_;

    /**
     * @brief Main loop.
     */
    void loop()
    {
        for (long int t = 0; !shutdown_requested_; t++)
        {
            ObservationType sensor_observation;
            try
            {
                sensor_observation = sensor_driver_->get_observation();
            }
            catch (const std::runtime_error &e)
            {
                std::cerr << e.what() << std::endl;
            }
            sensor_data_->observation->append(sensor_observation);
        }
    }
};

}  // namespace robot_interfaces
