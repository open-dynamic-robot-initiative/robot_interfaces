///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/loggable.hpp>
#include <robot_interfaces/monitored_robot_driver.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_driver.hpp>
#include <robot_interfaces/status.hpp>

namespace robot_interfaces
{

/**
 * @brief Timer to measure code execution time with "checkpoints"
 *
 * This timer is meant to be used for measuring execution time of a loop.  It
 * measures time between calls of the `start` method, so by calling this at the
 * beginning of the loop, you get the execution time of the full iteration.
 * Further, you can define "checkpoints" within the loop to measure time of
 * separate steps in the loop.  Call the `checkpoint` method after the code that
 * is associated with it.  For each checkpoint, the time elapsed since the last
 * checkpoint is measured (`start` counts as a checkpoint in this regard).
 *
 * Example:
 *
 * ~~~{.cpp}
 * CheckpointTimer<3> timer;
 *
 * while (condition)
 * {
 *     timer.start();
 *
 *     initialize()
 *     timer.checkpoint("initialize");
 *
 *     do();
 *     some();
 *     stuff();
 *     timer.checkpoint("do some stuff");
 *
 *     write_log();
 *     timer.checkpoint("logging");
 * }
 * ~~~
 *
 * @tparam NUM_CHECKPOINTS Number of checkpoints.
 * @tparam ENABLED Set to false, to disable timer.  Method calls will have no
 * effect (and should hopefully be optimized away by the compiler).
 */
template <size_t NUM_CHECKPOINTS, bool ENABLED = true>
class CheckpointTimer
{
public:

    //! @brief Start timer iteration.
    void start()
    {
        if (ENABLED)
        {
            timers_[0].tac_tic();
            current_checkpoint = 1;
            timers_[current_checkpoint].tic();
        }
    }

    /**
     * @brief Set checkpoint for time measurement.
     *
     * Measures time from the last call of start() or checkpoint() until this
     * call.  The given name is used when printing the results.
     *
     * @param checkpoint_name Name of the checkpoint (used for printing results)
     */
    void checkpoint(const std::string &checkpoint_name)
    {
        if (ENABLED)
        {
            timers_[current_checkpoint].tac();

            if (checkpoint_names_[current_checkpoint].empty())
            {
                checkpoint_names_[current_checkpoint] = checkpoint_name;
            }

            current_checkpoint++;
            if (current_checkpoint < timers_.size())
            {
                timers_[current_checkpoint].tic();
            }
        }
    }

    //! @brief Print results of time measurements.
    void print_statistics() const
    {
        if (ENABLED)
        {
            std::cout << "===== Total:" << std::endl;
            timers_[0].print_statistics();

            for (size_t i = 1; i < timers_.size(); i++)
            {
                std::cout << "----- " << checkpoint_names_[i] << std::endl;
                timers_[i].print_statistics();
            }
        }
    }

private:
    std::array<real_time_tools::Timer, NUM_CHECKPOINTS + 1> timers_;
    std::array<std::string, NUM_CHECKPOINTS + 1> checkpoint_names_;
    size_t current_checkpoint = 1;
};  // namespace robot_interfaces

/**
 * @brief Communication link between RobotDriver and RobotData.
 *
 * At each time-step, it gets the observation from the RobotDriver and
 * writes it to RobotData, and it takes the desired_action from RobotData
 * and applies it on the RobotDriver.
 *
 * @tparam Action
 * @tparam Observation
 */
template <typename Action, typename Observation>
class RobotBackend
{
public:
    /**
     * @param robot_driver  Driver instance for the actual robot.  This is
     *     internally wrapped in a MonitoredRobotDriver for increased
     * safety.
     * @param robot_data  Data is send to/retrieved from here.
     * @param max_action_duration_s  See MonitoredRobotDriver.
     * @param max_inter_action_duration_s  See MonitoredRobotDriver.
     */
    RobotBackend(
        std::shared_ptr<RobotDriver<Action, Observation>> robot_driver,
        std::shared_ptr<RobotData<Action, Observation, Status>> robot_data,
        const double max_action_duration_s,
        const double max_inter_action_duration_s)
        : robot_driver_(
              robot_driver, max_action_duration_s, max_inter_action_duration_s),
          robot_data_(robot_data),
          destructor_was_called_(false),
          max_action_repetitions_(0)
    {
        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
        thread_->create_realtime_thread(&RobotBackend::loop, this);
    }

    virtual ~RobotBackend()
    {
        destructor_was_called_ = true;
        thread_->join();
    }

    uint32_t get_max_action_repetitions()
    {
        return max_action_repetitions_;
    }

    /**
     * @brief Set how often an action is repeated if no new one is provided.
     *
     * If the next action is due to be executed but the user did not provide
     * one yet (i.e. there is no new action in the robot data time series),
     * the last action will be repeated by automatically adding it to the
     * time series again.
     *
     * Use this this method to specify how often the action shall be
     * repeated (default is 0, i.e. no repetition at all).  If this limit is
     * exceeded, the robot will be shut down and the RobotBackend stops.
     *
     * @param max_action_repetitions
     */
    void set_max_action_repetitions(const uint32_t &max_action_repetitions)
    {
        max_action_repetitions_ = max_action_repetitions;
    }

    void initialize()
    {
        robot_driver_.initialize();
    }

private:
    MonitoredRobotDriver<Action, Observation> robot_driver_;
    std::shared_ptr<RobotData<Action, Observation, Status>> robot_data_;
    std::atomic<bool> destructor_was_called_;

    /**
     * @brief Number of times the previous action is repeated if no new one
     *        is provided.
     */
    uint32_t max_action_repetitions_;

    CheckpointTimer<6, false> timer_;

    std::shared_ptr<real_time_tools::RealTimeThread> thread_;

    // control loop
    // ------------------------------------------------------------
    static void *loop(void *instance_pointer)
    {
        ((RobotBackend *)(instance_pointer))->loop();
        return nullptr;
    }

    /**
     * @brief Main loop.
     *
     * Iterate over robot_data_.desired_action and apply these actions to
     * the robot, and read the applied_action and the observation from the
     * robot and append them to the corresponding timeseries in robot_data_.
     */
    void loop()
    {
        real_time_tools::set_cpu_dma_latency(0);

        // wait until first desired_action was received
        // ----------------------------
        while (!destructor_was_called_ &&
               !robot_data_->desired_action->wait_for_timeindex(0, 0.1))
        {
        }

        for (long int t = 0; !destructor_was_called_; t++)
        {
            // TODO: figure out latency stuff!!

            timer_.start();

            // get latest observation from robot and append it to
            // robot_data_
            // --------
            Observation observation = robot_driver_.get_latest_observation();
            timer_.checkpoint("get observation");

            robot_data_->observation->append(observation);
            // TODO: for some reason this sometimes takes more than 2 ms
            // i think this may be due to a non-realtime thread blocking the
            // timeseries. this is in fact an issue, we might have to
            // duplicate all the timeseries and have a realtime thread
            // writing back and forth
            timer_.checkpoint("append observation");

            // if the robot has a finite max_inter_action_duration_s (not
            // NAN or infinite, meaning it requires receiving actions in
            // fixed time intervals), but robot_data_ has not received yet
            // the next action to apply, we optionally repeat the previous
            // action.
            Status status;
            if (std::isfinite(
                    robot_driver_.get_max_inter_action_duration_s()) &&
                robot_data_->desired_action->newest_timeindex() < t)
            {
                uint32_t action_repetitions =
                    robot_data_->status->newest_element().action_repetitions;

                if (action_repetitions < max_action_repetitions_)
                {
                    robot_data_->desired_action->append(
                        robot_data_->desired_action->newest_element());
                    status.action_repetitions = action_repetitions + 1;
                }
                else
                {
                    // No action provided and number of allowed repetitions
                    // of the previous action is exceeded --> Error
                    status.error_status = Status::ErrorStatus::BACKEND_ERROR;
                    status.error_message =
                        "Next action was not provided in time";
                }
            }

            std::string driver_error_msg = robot_driver_.get_error();
            if (!driver_error_msg.empty())
            {
                status.error_status = Status::ErrorStatus::DRIVER_ERROR;
                status.error_message = driver_error_msg;
            }

            robot_data_->status->append(status);

            // if there is an error, shut robot down and stop loop
            if (status.error_status != Status::ErrorStatus::NO_ERROR)
            {
                std::cerr << "Error: " << status.error_message
                          << "\nRobot is shut down." << std::endl;
                robot_driver_.shutdown();
                return;
            }
            timer_.checkpoint("status");

            // early exit if destructor has been called
            while (!robot_data_->desired_action->wait_for_timeindex(t, 0.1))
            {
                if (destructor_was_called_)
                {
                    // TODO should shut down robot?
                    return;
                }
            }
            Action desired_action = (*robot_data_->desired_action)[t];
            timer_.checkpoint("get action");

            Action applied_action = robot_driver_.apply_action(desired_action);
            timer_.checkpoint("apply action");

            robot_data_->applied_action->append(applied_action);
            timer_.checkpoint("append applied action");

            if (t % 5000 == 0)
            {
                timer_.print_statistics();
            }
        }
    }
};

}  // namespace robot_interfaces
