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

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/opencv_driver.hpp>
#include <robot_interfaces/sensor_data.hpp>
#include <robot_interfaces/sensor_driver.hpp>

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
    /**
     * @param sensor_driver  Driver instance for the sensor.
     * @param sensor_data  Data is sent to/retrieved from here.
     */

    SensorBackend(std::shared_ptr<SensorDriver<ObservationType>> sensor_driver,
                  std::shared_ptr<SensorData<ObservationType>> sensor_data)
        : sensor_driver_(sensor_driver),
          sensor_data_(sensor_data),
          destructor_was_called_(false)
    {
        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
        thread_->create_realtime_thread(&SensorBackend::loop, this);
    }

    virtual ~SensorBackend()
    {
        destructor_was_called_ = true;
        thread_->join();
    }

private:
    std::shared_ptr<SensorDriver<ObservationType>> sensor_driver_;
    std::shared_ptr<SensorData<ObservationType>> sensor_data_;

    bool destructor_was_called_;

    std::shared_ptr<real_time_tools::RealTimeThread> thread_;

    static void *loop(void *instance_pointer)
    {
        ((SensorBackend *)(instance_pointer))->loop();
        return nullptr;
    }

    /**
     * @brief Main loop.
     */

    void loop()
    {
        for (long int t = 0; !destructor_was_called_; t++)
        {
            bool flag = sensor_driver_->is_access_successful();
            if (flag)
            {
                ObservationType sensor_observation;
                sensor_observation = sensor_driver_->get_observation();
                sensor_data_->observation->append(sensor_observation);
            }
            else
            {
                std::cerr << "Cannot access the camera." << std::endl;
            }
        }
    }
};

}  // namespace robot_interfaces
