/**
 * @file
 * @brief Wrapper on the Pylon Driver to synchronise three pylon dependent
 * cameras.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 *
 */
#pragma once

#include <ctime>
#include <iostream>

#include <real_time_tools/timer.hpp>

#include <pylon/PylonIncludes.h>
#include <robot_interfaces/sensors/pylon_driver.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>
#include <robot_interfaces/sensors/tricamera_observation.hpp>

namespace robot_interfaces
{
/**
 * @brief Driver to create three instances of the PylonDriver
 * and get observations from them.
 */
class TriCameraDriver : public SensorDriver<TricameraObservation>
{
public:
    /**
     * @param device_id_1 device user id of first camera
     * @param device_id_2 likewise, the 2nd's
     * @param device_id_3 and the 3rd's
     */
    TriCameraDriver(const std::string& device_id_1,
               const std::string& device_id_2,
               const std::string& device_id_3)
        : cam_1(device_id_1), cam_2(device_id_2), cam_3(device_id_3)
    {
    }
    /**
     * @brief Get the latest observation from the three cameras
     * @return TricameraObservation
     */
    TricameraObservation get_observation()
    {
        TricameraObservation tricam_obs;

        tricam_obs.cam_array[0] = cam_1.get_observation();
        tricam_obs.cam_array[1] = cam_2.get_observation();
        tricam_obs.cam_array[2] = cam_3.get_observation();

        return tricam_obs;
    }

private:
    PylonDriver cam_1, cam_2, cam_3;
};

}  // namespace robot_interfaces