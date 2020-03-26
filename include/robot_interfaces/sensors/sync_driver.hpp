/**
 * @file
 * @brief Wrapper on the Pylon Driver to synchronise three pylon dependent cameras.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 *
 */
#pragma once

#include <ctime>
#include <iostream>

#include <real_time_tools/timer.hpp>

#include <robot_interfaces/sensors/tricamera_observation.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>
#include <robot_interfaces/sensors/pylon_driver.hpp>
#include <pylon/PylonIncludes.h>

namespace robot_interfaces
{
class SyncDriver : public SensorDriver<TricameraObservation>
{
public:
    // PylonDriver camera_1("cam_1");

    SyncDriver(const std::array<std::string, 3> device_ids) : cam_1(device_ids[0]), cam_2(device_ids[1]), cam_3(device_ids[3])
    {
        
    }

    TricameraObservation get_observation()
    {
        int index;
        TricameraObservation tricam_obs;

        tricam_obs.cam_array[0] = cam_1.get_observation();
        tricam_obs.cam_array[1] = cam_2.get_observation();
        tricam_obs.cam_array[2] = cam_3.get_observation();

        return tricam_obs;
    }

private:
    PylonDriver cam_1, cam_2, cam_3;    
    
};

} //namespace robot_interfaces