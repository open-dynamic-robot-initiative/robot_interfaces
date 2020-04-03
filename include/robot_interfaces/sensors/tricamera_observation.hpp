/**
 * @file
 * @brief Defines the observation structure to be used by any camera.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <robot_interfaces/sensors/camera_observation.hpp>

namespace robot_interfaces
{
/**
 * @brief Observation structure to store cv::Mat images with corresponding
 * timestamps.
 */
struct TriCameraObservation
{
    typedef std::array<CameraObservation, 3> CameraArray;
    CameraArray cameras;
};

}  // namespace robot_interfaces
