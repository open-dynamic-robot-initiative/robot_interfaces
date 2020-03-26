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

    PylonDriver camera_1("cam_1");
    PylonDriver camera_2("cam_2");
    PylonDriver camera_3("cam_3");

    TricameraObservation get_observation()
    {
        int index;
        TricameraObservation tricam_obs;
        for (index = 0; index < 3; index++)
        {
            tricam_obs.cam_array[index] = camera_1.get_observation();
        }
    }
    
}