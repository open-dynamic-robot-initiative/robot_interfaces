///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <algorithm>
#include <cmath>

#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>

#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>

namespace robot_interfaces
{
template <typename Type>
using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;

typedef Timeseries<int>::Index TimeIndex;

/**
 * @brief Communication link between RobotData and the user.
 *
 * Takes care of communication between the RobotData and the user. It is just a
 * thin wrapper around RobotData to facilitate interaction and also to make sure
 * the user cannot use RobotData in incorrect ways.
 *
 * @tparam Action
 * @tparam Observation
 */
template <typename Action, typename Observation>
class RobotFrontend
{
public:
    typedef Timeseries<int>::Timestamp TimeStamp;

    RobotFrontend(
        std::shared_ptr<RobotData<Action, Observation, Status>> robot_data)
        : robot_data_(robot_data)
    {
    }

    Observation get_observation(const TimeIndex &t)
    {
        return (*robot_data_->observation)[t];
    }
    Action get_desired_action(const TimeIndex &t)
    {
        return (*robot_data_->desired_action)[t];
    }
    Action get_applied_action(const TimeIndex &t)
    {
        return (*robot_data_->applied_action)[t];
    }
    TimeStamp get_time_stamp_ms(const TimeIndex &t)
    {
        return robot_data_->observation->timestamp_ms(t);
    }
    TimeIndex get_current_timeindex()
    {
        return robot_data_->observation->newest_timeindex();
    }

    TimeIndex append_desired_action(const Action &desired_action)
    {
        // since the timeseries has a finite memory, we need to make sure that
        // by appending new actions we do not forget about actions which have
        // not been applied yet
        if (robot_data_->desired_action->length() ==
                robot_data_->desired_action->max_length() &&
            robot_data_->desired_action->oldest_timeindex() ==  // FIXME >=
                get_current_timeindex())
        {
            std::cout
                << "you have been appending actions too fast, waiting for "
                   "RobotBackend to catch up with executing actions."
                << std::endl;
            wait_until_timeindex(
                robot_data_->desired_action->oldest_timeindex() + 1);
        }

        robot_data_->desired_action->append(desired_action);
        return robot_data_->desired_action->newest_timeindex();
    }

    void wait_until_timeindex(const TimeIndex &t)
    {
        robot_data_->observation->timestamp_ms(t);
    }

private:
    std::shared_ptr<RobotData<Action, Observation, Status>> robot_data_;
};

}  // namespace robot_interfaces
