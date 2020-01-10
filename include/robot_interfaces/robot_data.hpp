///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <time_series/time_series.hpp>

namespace robot_interfaces
{
template <typename Type>
using Timeseries = time_series::TimeSeries<Type>;

/**
 * @brief Contains all the input and output data of the robot.
 *
 * This means the
 * - desired_action which was requested by the robot user
 * - applied_action which was actually applied and may not be
 *                  and may not be identical to desired_action
 *                  for safety reasons
 * - observation made by the robot
 * - status which keeps track of some timing issues (may still change).
 *
 * See this graph to understand how they relate to each other precisely
 * in terms of time:
 *
 * |------ t = 0 ------|------ t = 1 ------|
 * |----- action0 -----|----- action1 -----|
 * o                   o                   o
 * b                   b                   b
 * s                   s                   s
 * 0                   1                   2
 *
 *
 * @tparam Action
 * @tparam Observation
 * @tparam Status
 */

template <typename Action, typename Observation, typename Status>
class RobotData
{
public:
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;

    RobotData(size_t history_length = 1000,
              bool use_shared_memory = false,
              // suppress unused warning (will be used in the future)
              __attribute__((unused)) std::string shared_memory_address = "")
    {
        if (use_shared_memory)
        {
            std::cout << "shared memory robot data is not implemented yet"
                      << std::endl;
            exit(-1);

            // TODO: here we should check if the shared memory at that
            // address already exists, otherwise we create it.
            // we will also have to update timeseries such as to handle
            // serialization internally (it will simply assume that the
            // templated class has a method called serialize() and
            // from_serialized())
        }
        else
        {
            desired_action =
                std::make_shared<Timeseries<Action>>(history_length);
            applied_action =
                std::make_shared<Timeseries<Action>>(history_length);
            observation =
                std::make_shared<Timeseries<Observation>>(history_length);
            status = std::make_shared<Timeseries<Status>>(history_length);
        }
    }

public:
    Ptr<Timeseries<Action>> desired_action;
    Ptr<Timeseries<Action>> applied_action;
    Ptr<Timeseries<Observation>> observation;
    Ptr<Timeseries<Status>> status;
};

}  // namespace robot_interfaces
