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

#include <time_series/time_series.hpp>

#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/status.hpp>

namespace robot_interfaces
{
template <typename Type>
using Timeseries = time_series::TimeSeries<Type>;

typedef time_series::Index TimeIndex;

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
    typedef time_series::Timestamp TimeStamp;

    RobotFrontend(std::shared_ptr<RobotData<Action, Observation>> robot_data)
        : robot_data_(robot_data)
    {
    }

    Observation get_observation(const TimeIndex &t) const
    {
        return (*robot_data_->observation)[t];
    }
    Action get_desired_action(const TimeIndex &t) const
    {
        return (*robot_data_->desired_action)[t];
    }
    Action get_applied_action(const TimeIndex &t) const
    {
        return (*robot_data_->applied_action)[t];
    }
    Status get_status(const TimeIndex &t) const
    {
        return (*robot_data_->status)[t];
    }

    //! @deprecated Use get_timestamp_ms instead
    [[deprecated]] TimeStamp get_time_stamp_ms(const TimeIndex &t) const
    {
        return get_timestamp_ms(t);
    }
    TimeStamp get_timestamp_ms(const TimeIndex &t) const
    {
        return robot_data_->observation->timestamp_ms(t);
    }
    TimeIndex get_current_timeindex() const
    {
        return robot_data_->observation->newest_timeindex();
    }

    TimeIndex append_desired_action(const Action &desired_action)
    {
        // check error state. do not allow appending actions if there is an
        // error
        if (robot_data_->status->length() > 0)
        {
            const Status status = robot_data_->status->newest_element();
            switch (status.error_status)
            {
                case Status::ErrorStatus::NO_ERROR:
                    break;
                case Status::ErrorStatus::DRIVER_ERROR:
                    throw std::runtime_error("Driver Error: " +
                                             status.error_message);
                case Status::ErrorStatus::BACKEND_ERROR:
                    throw std::runtime_error("Backend Error: " +
                                             status.error_message);
                default:
                    throw std::runtime_error("Unknown Error: " +
                                             status.error_message);
            }
        }

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

    void wait_until_timeindex(const TimeIndex &t) const
    {
        robot_data_->observation->timestamp_ms(t);
    }

protected:
    std::shared_ptr<RobotData<Action, Observation>> robot_data_;
};

}  // namespace robot_interfaces
