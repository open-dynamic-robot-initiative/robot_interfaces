/**
 * @file
 * @brief Connects the driver with sensor data
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <cmath>
#include <thread>

#include <real_time_tools/thread.hpp>

#include <robot_interfaces/sensors/sensor_data.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>
#include <robot_interfaces/utils.hpp>

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
template <typename ObservationType, typename InfoType = None>
class SensorBackend
{
public:
    typedef std::shared_ptr<SensorBackend<ObservationType, InfoType>> Ptr;
    typedef std::shared_ptr<const SensorBackend<ObservationType, InfoType>>
        ConstPtr;

    /**
     * @param sensor_driver  Driver instance for the sensor.
     * @param sensor_data  Data is sent to/retrieved from here.
     */
    SensorBackend(
        std::shared_ptr<SensorDriver<ObservationType, InfoType>> sensor_driver,
        std::shared_ptr<SensorData<ObservationType, InfoType>> sensor_data)
        : sensor_driver_(sensor_driver),
          sensor_data_(sensor_data),
          loop_is_running_(false),
          shutdown_requested_(false)
    {
        // populate the sensor information field
        InfoType info = sensor_driver_->get_sensor_info();
        sensor_data_->sensor_info->append(info);

        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
        loop_is_running_ = true;
        thread_->create_realtime_thread(&SensorBackend::loop, this);
    }

    // reinstate the implicit move constructor
    // See https://stackoverflow.com/a/27474070
    SensorBackend(SensorBackend &&) = default;

    //! @brief Stop the backend thread.
    void shutdown()
    {
        shutdown_requested_ = true;

        while (loop_is_running_)
        {
            real_time_tools::Timer::sleep_microseconds(100000);
        }
    }

    virtual ~SensorBackend()
    {
        shutdown();
    }

private:
    std::shared_ptr<SensorDriver<ObservationType, InfoType>> sensor_driver_;
    std::shared_ptr<SensorData<ObservationType, InfoType>> sensor_data_;

    //! @brief Indicates if the background loop is still running.
    std::atomic<bool> loop_is_running_;

    bool shutdown_requested_;

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

        loop_is_running_ = false;
    }
};

}  // namespace robot_interfaces
