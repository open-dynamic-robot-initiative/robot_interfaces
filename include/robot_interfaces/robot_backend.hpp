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
#include <cstdlib>

#include <pybind11/embed.h>

#include <real_time_tools/checkpoint_timer.hpp>
#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>

#include <signal_handler/signal_handler.hpp>

#include <robot_interfaces/loggable.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_driver.hpp>
#include <robot_interfaces/status.hpp>

namespace robot_interfaces
{
//! @brief Possible termination reasons of a @ref RobotBackend.
enum RobotBackendTerminationReason : int
{
    // ## non-failure cases (use positive numbers here)

    //! Backend is still running.
    NOT_TERMINATED = 0,
    //! Shutdown requested for non-failure reason (e.g. by SIGINT).
    SHUTDOWN_REQUESTED = 1,
    //! Maximum number of actions was reached.
    MAXIMUM_NUMBER_OF_ACTIONS_REACHED = 2,

    // ## failure cases (use negative numbers here)

    //! Some error in the driver.
    DRIVER_ERROR = -1,
    //! First action timeout was triggered.
    FIRST_ACTION_TIMEOUT = -2,
    //! Next action timeout was triggered.
    NEXT_ACTION_TIMEOUT = -3
};

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
    typedef std::shared_ptr<RobotBackend<Action, Observation>> Ptr;
    typedef std::shared_ptr<const RobotBackend<Action, Observation>> ConstPtr;

    /**
     * @param robot_driver  Driver instance for the actual robot.
     * @param robot_data  Data is send to/retrieved from here.
     * @param real_time_mode  Enable/disable real-time mode.  In real-time mode,
     *     the backend will repeat previous actions if the new one is not
     *     provided in time or fail with an error if the allowed number of
     *     repetitions is exceeded.  In non-real-time mode, it will simply block
     *     and wait until the action is provided.
     * @param first_action_timeout  See RobotBackend::first_action_timeout_.
     * @param max_number_of_actions  See RobotBackend::max_number_of_actions_.
     */
    RobotBackend(std::shared_ptr<RobotDriver<Action, Observation>> robot_driver,
                 std::shared_ptr<RobotData<Action, Observation>> robot_data,
                 const bool real_time_mode = true,
                 const double first_action_timeout =
                     std::numeric_limits<double>::infinity(),
                 const uint32_t max_number_of_actions = 0)
        : robot_driver_(robot_driver),
          robot_data_(robot_data),
          real_time_mode_(real_time_mode),
          first_action_timeout_(first_action_timeout),
          max_number_of_actions_(max_number_of_actions),
          is_shutdown_requested_(false),
          loop_is_running_(false),
          is_initialized_(false),
          max_action_repetitions_(0),
          termination_reason_(RobotBackendTerminationReason::NOT_TERMINATED)
    {
        signal_handler::SignalHandler::initialize();

        if (max_number_of_actions_ > 0)
        {
            frequency_timer_.set_memory_size(max_number_of_actions_);
        }

        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
        loop_is_running_ = true;
        thread_->create_realtime_thread(&RobotBackend::loop, this);
    }

    virtual ~RobotBackend()
    {
        // pybind11::gil_scoped_release causes a segfault when the class is used
        // directly from C++ (i.e. no Python interpreter running).
        // Best workaround found so far is to explicitly check if Python is
        // initialized or not...
        // See https://github.com/pybind/pybind11/issues/2177
        if (Py_IsInitialized())
        {
            // Release the GIL when destructing the backend, as otherwise the
            // program will get stuck in a dead lock in case the driver needs to
            // run some Python code.
            pybind11::gil_scoped_release release;

            request_shutdown();
            thread_->join();
        }
        else
        {
            request_shutdown();
            thread_->join();
        }
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
     * **Note:** This is ignored in non-real-time mode.
     *
     * @param max_action_repetitions
     */
    void set_max_action_repetitions(const uint32_t &max_action_repetitions)
    {
        max_action_repetitions_ = max_action_repetitions;
    }

    void initialize()
    {
        robot_driver_->initialize();
        is_initialized_ = true;
    }

    /**
     * @brief Request shutdown of the backend loop.
     *
     * The loop may take some time to actually terminate after calling this
     * function. Use wait_until_terminated() to ensure it has really terminated.
     */
    void request_shutdown()
    {
        is_shutdown_requested_ = true;
    }

    /**
     * @brief Wait until the first desired action is received.
     */
    void wait_until_first_action() const
    {
        while (!has_shutdown_request() &&
               !robot_data_->desired_action->wait_for_timeindex(0, 0.1))
        {
        }
    }

    /**
     * @brief Wait until the backend loop terminates.
     * @return Termination code (see @ref RobotBackendTerminationReason).
     */
    int wait_until_terminated() const
    {
        while (loop_is_running_)
        {
            real_time_tools::Timer::sleep_microseconds(100000);
        }

        return termination_reason_;
    }

    /**
     * @brief Check if the backend loop is still running.
     * @return True if the loop is still running.
     */
    bool is_running() const
    {
        return loop_is_running_;
    }

    //! @brief Get the termination reason
    int get_termination_reason() const
    {
        return termination_reason_;
    }

private:
    std::shared_ptr<RobotDriver<Action, Observation>> robot_driver_;
    std::shared_ptr<RobotData<Action, Observation>> robot_data_;

    /**
     * @brief Enable/disable real time mode.
     *
     * If real time mode is enabled (true), the back end expects new actions to
     * be provided in time by the user.  If this does not happen, the last
     * received action is repeated until the configured number of repetitions is
     * exceeded in which case it stops with an error.
     *
     * If real time mode is disabled (false), the back-end loop blocks and waits
     * for the next action if it is not provided in time.
     *
     * @see max_action_repetitions_
     */
    const bool real_time_mode_;

    /**
     * @brief Timeout for the first action to arrive.
     *
     * Timeout for the time between starting the backend loop and receiving the
     * first action from the user.  If exceeded, the backend shuts down.
     * Set to infinity to disable the timeout.
     */
    const double first_action_timeout_;

    /**
     * @brief Maximum number of actions that are executed by the backend.
     *
     * If set to a value greater than zero, the backend will automatically shut
     * down after the specified number of actions is executed.
     */
    const uint32_t max_number_of_actions_;

    /**
     * @brief Set to true when shutdown is requested.
     *
     * This is used to notify the background loop about requested shutdown, so
     * it terminates itself.
     */
    std::atomic<bool> is_shutdown_requested_;

    //! @brief Indicates if the background loop is still running.
    std::atomic<bool> loop_is_running_;

    //! @brief Indicates if initialize() has been executed
    std::atomic<bool> is_initialized_;

    /**
     * @brief Number of times the previous action is repeated if no new one
     *        is provided.
     */
    uint32_t max_action_repetitions_;

    real_time_tools::CheckpointTimer<6, false> timer_;

    //! @brief Measure the duration of the control loop (for analysing time
    //!        consistency).
    real_time_tools::Timer frequency_timer_;

    std::shared_ptr<real_time_tools::RealTimeThread> thread_;

    std::atomic<int> termination_reason_;

    bool has_shutdown_request() const
    {
        return is_shutdown_requested_ ||
               signal_handler::SignalHandler::has_received_sigint();
    }

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
        //
        // IMPORTANT: This method is run in a real-time thread.  This means no
        // dynamic memory allocation should happen after the initialisation
        // phase.  This also means not to use dynamic std containers like
        // std::vector or std::string!
        // Also no IO operations should be done here (except in error cases when
        // the loop is aborted anyway).
        //

        const double start_time =
            real_time_tools::Timer::get_current_time_sec();

        while (!has_shutdown_request() && !is_initialized_)
        {
            real_time_tools::Timer::sleep_ms(0.1);
        }

        // wait until first desired_action was received
        // ----------------------------
        while (!has_shutdown_request() &&
               robot_data_->desired_action->is_empty())
        {
            const double now = real_time_tools::Timer::get_current_time_sec();
            if (now - start_time > first_action_timeout_)
            {
                Status status;
                status.set_error(Status::ErrorStatus::BACKEND_ERROR,
                                 "First action was not provided in time");
                termination_reason_ =
                    RobotBackendTerminationReason::FIRST_ACTION_TIMEOUT;

                robot_data_->status->append(status);

                std::cerr << "Error: " << status.get_error_message()
                          << "\nRobot is shut down." << std::endl;

                request_shutdown();
                break;
            }

            // apply 'idle action' while waiting
            Action idle_action = robot_driver_->get_idle_action();
            robot_driver_->apply_action(idle_action);
        }

        for (long int t = 0; !has_shutdown_request(); t++)
        {
            frequency_timer_.tac_tic();

            // TODO: figure out latency stuff!!

            Status status;

            if (max_number_of_actions_ > 0 && t >= max_number_of_actions_)
            {
                // TODO this is not really an error
                status.set_error(Status::ErrorStatus::BACKEND_ERROR,
                                 "Maximum number of actions reached.");
                termination_reason_ = RobotBackendTerminationReason::
                    MAXIMUM_NUMBER_OF_ACTIONS_REACHED;
            }

            timer_.start();

            // get latest observation from robot and append it to robot_data_
            Observation observation = robot_driver_->get_latest_observation();
            timer_.checkpoint("get observation");

            robot_data_->observation->append(observation);
            // TODO: for some reason this sometimes takes more than 2 ms
            // i think this may be due to a non-realtime thread blocking the
            // timeseries. this is in fact an issue, we might have to
            // duplicate all the timeseries and have a realtime thread
            // writing back and forth
            timer_.checkpoint("append observation");

            // If real time mode is enabled the next action needs to be provided
            // in time.  If this is not the case, optionally repeat the previous
            // action or raise an error.
            if (real_time_mode_ &&
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
                    status.set_error(Status::ErrorStatus::BACKEND_ERROR,
                                     "Next action was not provided in time");
                    termination_reason_ =
                        RobotBackendTerminationReason::NEXT_ACTION_TIMEOUT;
                }
            }

            // FIXME don't use std::string in real-time critical code!
            std::string driver_error_msg = robot_driver_->get_error();
            if (!driver_error_msg.empty())
            {
                status.set_error(Status::ErrorStatus::DRIVER_ERROR,
                                 driver_error_msg);
                termination_reason_ =
                    RobotBackendTerminationReason::DRIVER_ERROR;
            }

            robot_data_->status->append(status);

            // if there is an error, shut robot down and stop loop
            if (status.error_status != Status::ErrorStatus::NO_ERROR)
            {
                std::cerr << "Error: " << status.get_error_message()
                          << "\nRobot is shut down." << std::endl;
                break;
            }
            timer_.checkpoint("status");

            // early exit if destructor has been called
            while (!has_shutdown_request() &&
                   !robot_data_->desired_action->wait_for_timeindex(t, 0.1))
            {
            }
            if (has_shutdown_request())
            {
                break;
            }

            Action desired_action = (*robot_data_->desired_action)[t];
            timer_.checkpoint("get action");

            Action applied_action = robot_driver_->apply_action(desired_action);
            timer_.checkpoint("apply action");

            robot_data_->applied_action->append(applied_action);
            timer_.checkpoint("append applied action");

            if (t % 5000 == 0 && t > 0)
            {
                timer_.print_statistics();
            }
        }

        robot_driver_->shutdown();

        // print frequency statistics and dump full log if environment variable
        // ROBOT_BACKEND_TIME_LOG_FILE is set
        frequency_timer_.print_statistics();
        const char *time_log_file = std::getenv("ROBOT_BACKEND_TIME_LOG_FILE");
        if (time_log_file != nullptr and time_log_file[0] != '\0')
        {
            std::cout << "Write time log to " << time_log_file << std::endl;
            frequency_timer_.dump_measurements(time_log_file);
        }

        // If no specific termination reason was set, assume that the shutdown
        // was requested from outside.
        if (termination_reason_ ==
            RobotBackendTerminationReason::NOT_TERMINATED)
        {
            termination_reason_ =
                RobotBackendTerminationReason::SHUTDOWN_REQUESTED;
        }

        loop_is_running_ = false;
    }
};

}  // namespace robot_interfaces
