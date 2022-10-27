/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 *            Gesellschaft
 *
 * @brief RobotData classes for both single- and multi-process applications.
 */

#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <time_series/multiprocess_time_series.hpp>
#include <time_series/time_series.hpp>

#include "status.hpp"

namespace robot_interfaces
{
/**
 * @brief Contains all the input and output data of the robot.
 *
 * This means the
 * - `desired_action` which was requested by the robot user
 * - `applied_action` which was actually applied and may not be
 *                  and may not be identical to desired_action
 *                  for safety reasons
 * - `observation` made by the robot
 * - `status` which keeps track of timing issues and errors.
 *
 * See this graph to understand how they relate to each other precisely
 * in terms of time:
 *
 * @verbatim
   |------ t = 0 ------|------ t = 1 ------|
   |----- action0 -----|----- action1 -----|
   o                   o                   o
   b                   b                   b
   s                   s                   s
   0                   1                   2
   @endverbatim
 *
 * @tparam Action Type of the actions.
 * @tparam Observation Type of the observations.
 */
template <typename Action, typename Observation>
class RobotData
{
public:
    typedef std::shared_ptr<RobotData<Action, Observation>> Ptr;
    typedef std::shared_ptr<const RobotData<Action, Observation>> ConstPtr;

    //! @brief Time series of the desired actions.
    std::shared_ptr<time_series::TimeSeriesInterface<Action>> desired_action;
    //! @brief Time series of the actually applied actions (due to safety
    //         checks).
    std::shared_ptr<time_series::TimeSeriesInterface<Action>> applied_action;
    //! @brief Time series of the observations retrieved from the robot.
    std::shared_ptr<time_series::TimeSeriesInterface<Observation>> observation;
    //! @brief Time series of status messages.
    std::shared_ptr<time_series::TimeSeriesInterface<Status>> status;

protected:
    // make constructor protected to prevent instantiation of the base class
    RobotData(){};
};

/**
 * @brief RobotData instance using single process time series.
 *
 * Use this class if all modules accessing the data are running in the same
 * process.  If modules run in separate processes, use MultiProcessRobotData
 * instead.
 *
 * @copydoc RobotData
 * @see MultiProcessRobotData
 */
template <typename Action, typename Observation>
class SingleProcessRobotData : public RobotData<Action, Observation>
{
public:
    /**
     * @brief Construct the time series for the robot data.
     *
     * @param history_length History length of the time series.
     */
    SingleProcessRobotData(size_t history_length = 1000)
    {
        std::cout << "Using single process time series." << std::endl;
        this->desired_action =
            std::make_shared<time_series::TimeSeries<Action>>(history_length);
        this->applied_action =
            std::make_shared<time_series::TimeSeries<Action>>(history_length);
        this->observation =
            std::make_shared<time_series::TimeSeries<Observation>>(
                history_length);
        this->status =
            std::make_shared<time_series::TimeSeries<Status>>(history_length);
    }
};

/**
 * @brief RobotData instance using multi process time series.
 *
 * Use this class if modules accessing the data are running in separate
 * processes.  When all modules run as threads in the same process, this class
 * can be used as well, however, SingleProcessRobotData might be more efficient
 * in that case.
 *
 * @copydoc RobotData
 * @see SingleProcessRobotData
 */
template <typename Action, typename Observation>
class MultiProcessRobotData : public RobotData<Action, Observation>
{
public:
    /**
     * @brief Construct the time series for the robot data.
     *
     * @param shared_memory_id_prefix Prefix for the shared memory IDs.  Since
     *     each time series needs its own memory ID, the given value is used as
     *     prefix and unique suffixes are appended.  Make sure to use a prefix
     *     that cannot lead to name collisions on your system.
     * @param is_master If set to true, this instance will clear the shared
     *     memory on construction and destruction.  Only one instance should
     *     act as master in a multi-process setup.
     * @param history_length History length of the time series.  Ignored if
     *     `is_master == false`.
     *
     * @todo Make this constructor protected and implement factory methods like
     *     in MultiprocessTimeSeries..
     */
    MultiProcessRobotData(const std::string &shared_memory_id_prefix,
                          bool is_master,
                          size_t history_length = 1000)
    {
        std::cout << "Using multi process time series." << std::endl;

        // each time series needs its own shared memory ID, so add unique
        // suffixes to the given ID.
        const std::string id_desired_action =
            shared_memory_id_prefix + "_desired_action";
        const std::string id_applied_action =
            shared_memory_id_prefix + "_applied_action";
        const std::string id_observation =
            shared_memory_id_prefix + "_observation";
        const std::string id_status = shared_memory_id_prefix + "_status";

        typedef time_series::MultiprocessTimeSeries<Action> TS_Action;
        typedef time_series::MultiprocessTimeSeries<Observation> TS_Observation;
        typedef time_series::MultiprocessTimeSeries<Status> TS_Status;

        if (is_master)
        {
            // the master instance is in charge of cleaning the memory
            time_series::clear_memory(id_desired_action);
            time_series::clear_memory(id_applied_action);
            time_series::clear_memory(id_observation);
            time_series::clear_memory(id_status);

            this->desired_action =
                TS_Action::create_leader_ptr(id_desired_action, history_length);
            this->applied_action =
                TS_Action::create_leader_ptr(id_applied_action, history_length);
            this->observation = TS_Observation::create_leader_ptr(
                id_observation, history_length);
            this->status =
                TS_Status::create_leader_ptr(id_status, history_length);
        }
        else
        {
            this->desired_action =
                TS_Action::create_follower_ptr(id_desired_action);
            this->applied_action =
                TS_Action::create_follower_ptr(id_applied_action);
            this->observation =
                TS_Observation::create_follower_ptr(id_observation);
            this->status = TS_Status::create_follower_ptr(id_status);
        }
    }
};

}  // namespace robot_interfaces
