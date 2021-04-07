/**
 * \file
 * \copyright Copyright (c) 2020, Max Planck Gesellschaft.
 */
#pragma once

#include <time_series/interface.hpp>

#include <robot_interfaces/status.hpp>

namespace robot_interfaces
{
/**
 * @brief Robot log entry used for binary log format.
 *
 * Contains all the robot data of one time step.
 */
template <typename Action, typename Observation, typename Status_t = Status>
struct RobotLogEntry
{
    time_series::Index timeindex;
    time_series::Timestamp timestamp;
    Status_t status;
    Observation observation;
    Action desired_action;
    Action applied_action;

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(timeindex,
                timestamp,
                status,
                observation,
                desired_action,
                applied_action);
    }
};
}  // namespace robot_interfaces
