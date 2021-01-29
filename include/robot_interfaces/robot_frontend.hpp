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
#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/status.hpp>
#include <time_series/time_series.hpp>

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

    /**
     * @brief Get observation of time step t.
     *
     * @param t Index of the time step.  If t is in the future, this method will
     *     block and wait.
     * @return The observation of time step t.
     * @throws std::invalid_argument if t is too old and not in the time series
     *     buffer anymore.
     */
    Observation get_observation(const TimeIndex &t) const
    {
        return (*robot_data_->observation)[t];
    }

    /**
     * @brief Get the desired action of time step t.
     *
     * The desired action is the action as it is passed by the user in @ref
     * append_desired_action.
     *
     * @param t Index of the time step.  If t is in the future, this method will
     *     block and wait.
     * @return The desired action of time step t.
     * @throws std::invalid_argument if t is too old and not in the time series
     *     buffer anymore.
     */
    Action get_desired_action(const TimeIndex &t) const
    {
        return (*robot_data_->desired_action)[t];
    }

    /**
     * @brief Get the applied action of time step t.
     *
     * The applied action is the one that was actually applied to the robot
     * based on the desired action of that time step.  It may differ from the
     * desired one e.g. due to some safety checks which limit the maximum
     * torque.  If and how the action is modified depends on the implementation
     * of the @ref RobotDriver.
     *
     * @param t Index of the time step.  If t is in the future, this method will
     *     block and wait.
     * @return The applied action of time step t.
     * @throws std::invalid_argument if t is too old and not in the time series
     *     buffer anymore.
     */
    Action get_applied_action(const TimeIndex &t) const
    {
        return (*robot_data_->applied_action)[t];
    }
    Status get_status(const TimeIndex &t) const
    {
        return (*robot_data_->status)[t];
    }

    //! @deprecated Use get_timestamp_ms instead
    [[deprecated]] TimeStamp get_time_stamp_ms(const TimeIndex &t) const {
        return get_timestamp_ms(t);
    }

    /**
     * @brief Get the timestamp of time step t.
     *
     * @param t Index of the time step.  If t is in the future, this method will
     *     block and wait.
     * @return Timestamp of time step t.
     * @throws std::invalid_argument if t is too old and not in the time series
     *     buffer anymore.
     */
    TimeStamp get_timestamp_ms(const TimeIndex &t) const
    {
        return robot_data_->observation->timestamp_ms(t);
    }

    /**
     * @brief Get the current time index.
     *
     * @return The latest time index for which observations are available.
     */
    TimeIndex get_current_timeindex() const
    {
        return robot_data_->observation->newest_timeindex();
    }

    /**
     * @brief Append a desired action to the action time series.
     *
     * This will append an action to the "desired actions" time series.  Note
     * that this does not block until the action is actually executed. The time
     * series acts like a queue from which the @ref RobotBackend takes the
     * actions one by one to send them to the actual robot.  It is possible to
     * call this method multiple times in a row to already provide actions for
     * the next time steps.
     *
     * The time step at which the given action will be applied is returned by
     * this method.
     *
     * @param desired_action  The action that shall be applied on the robot.
     *     Note that the actually applied action might be different depending on
     *     the implementation of the @ref RobotDriver (see @ref
     *     get_applied_action).
     * @return Time step at which the action will be applied.
     */
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
                                             status.get_error_message());
                case Status::ErrorStatus::BACKEND_ERROR:
                    throw std::runtime_error("Backend Error: " +
                                             status.get_error_message());
                default:
                    throw std::runtime_error("Unknown Error: " +
                                             status.get_error_message());
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

    /**
     * @brief Wait until the specified time step is reached.
     *
     * @param t Time step until which is waited.
     * @throws std::invalid_argument if t is too old and not in the time series
     *     buffer anymore.
     */
    void wait_until_timeindex(const TimeIndex &t) const
    {
        robot_data_->observation->timestamp_ms(t);
    }

protected:
    std::shared_ptr<RobotData<Action, Observation>> robot_data_;
};

}  // namespace robot_interfaces
