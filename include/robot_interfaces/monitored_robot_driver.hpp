///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <iostream>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/robot_driver.hpp>

namespace robot_interfaces
{
/**
 * @brief Wrapper for RobotDriver that monitors timing.
 *
 * Takes a RobotDriver instance as input and forwards all method calls to it.  A
 * background loop monitors timing of actions to ensure the following
 * constraints:
 *
 *   1. The execution of an action does not take longer than
 *      `max_action_duration_s_` seconds.
 *   2. The time interval between termination of the previous action and
 *      receival of the next one (through `apply_action()`) does not exceed
 *      `max_inter_action_duration_s_`.
 *
 * If these timing constraints are not satisfied, the robot will be shutdown,
 * and no more actions from the outside will be accepted.
 *
 * This wrapper also makes sure that the `shutdown()` method of the given
 * RobotDriver is called when wrapper is destroyed, so the robot should always
 * be left in a safe state.
 *
 * @tparam Action
 * @tparam Observation
 */
template <typename Action, typename Observation>
class MonitoredRobotDriver : public RobotDriver<Action, Observation>
{
public:
    typedef std::shared_ptr<RobotDriver<Action, Observation>> RobotDriverPtr;

    /**
     * @brief Starts a thread for monitoring timing of action execution.
     *
     * @param robot_driver  The actual robot driver instance.
     * @param max_action_duration_s  Maximum time allowed for an action to be
     *     executed.
     * @param max_inter_action_duration_s  Maximum time allowed between end of
     *     the previous action and receival of the next one.
     */
    MonitoredRobotDriver(RobotDriverPtr robot_driver,
                         const double max_action_duration_s,
                         const double max_inter_action_duration_s)
        : robot_driver_(robot_driver),
          max_action_duration_s_(max_action_duration_s),
          max_inter_action_duration_s_(max_inter_action_duration_s),
          is_shutdown_(false),
          action_start_logger_(1000),
          action_end_logger_(1000)
    {
        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
        thread_->create_realtime_thread(&MonitoredRobotDriver::loop, this);
    }

    /**
     * @brief Shuts down the robot and stops the monitoring thread.
     */
    ~MonitoredRobotDriver()
    {
        shutdown();
        thread_->join();
    }

    double get_max_inter_action_duration_s()
    {
        return max_inter_action_duration_s_;
    }

    /**
     * @brief Apply desired action on the robot.
     *
     * If the robot is shut down, no more actions will be applied (the method
     * will just ignore them silently.
     *
     * @param desired_action  The desired action.
     * @return  The action that is actually applied on the robot (may differ
     *     from desired action due to safety limitations).
     */
    virtual Action apply_action(const Action &desired_action) final
    {
        if (is_shutdown_)
        {
            // FIXME I don't think it makes sense to return the desired action
            // in case of shutdown.  Shouldn't it rather be s.th. like Zero()?
            return desired_action;
        }
        action_start_logger_.append(true);
        Action applied_action = robot_driver_->apply_action(desired_action);
        action_end_logger_.append(true);
        return applied_action;
    }

    virtual void initialize()
    {
        robot_driver_->initialize();
    }

    virtual Observation get_latest_observation()
    {
        return robot_driver_->get_latest_observation();
    }

    /**
     * @brief Shut down the robot safely.
     *
     * After shutdown, actions sent by the user are ignored.
     */
    virtual void shutdown() final
    {
        if (!is_shutdown_)
        {
            is_shutdown_ = true;
            robot_driver_->shutdown();
        }
    }

private:
    //! \brief The actual robot driver.
    RobotDriverPtr robot_driver_;
    //! \brief Max. time for executing an action.
    double max_action_duration_s_;
    //! \brief Max. idle time between actions.
    double max_inter_action_duration_s_;

    //! \brief Whether shutdown was initiated.
    bool is_shutdown_;  // TODO: should be atomic

    real_time_tools::ThreadsafeTimeseries<bool> action_start_logger_;
    real_time_tools::ThreadsafeTimeseries<bool> action_end_logger_;

    std::shared_ptr<real_time_tools::RealTimeThread> thread_;

    /**
     * @brief Monitor the timing of action execution.
     *
     * If one of the timing constrains is violated, the robot is immediately
     * shut down.
     */
    void loop()
    {
        real_time_tools::set_cpu_dma_latency(0);

        // wait for the first data
        while (!is_shutdown_ &&
               !action_start_logger_.wait_for_timeindex(0, 0.1))
        {
        }

        // loop until shutdown and monitor action timing
        for (size_t t = 0; !is_shutdown_; t++)
        {
	  bool action_has_ended_on_time;
	  if (max_action_duration_s_ >0)
	    {
	      action_has_ended_on_time = action_end_logger_.wait_for_timeindex(t,
									       max_action_duration_s_);
	    }
	  else
	    {
	      action_has_ended_on_time = action_end_logger_.wait_for_timeindex(t);
	    }
	    
            if (!action_has_ended_on_time)
            {
                std::cout
                    << "Action did not end on time, shutting down. Any further "
                       "actions will be ignored."
                    << std::endl;
                shutdown();
                return;
            }

            bool action_has_started_on_time;
	    if(max_inter_action_duration_s_>0)
	      action_has_started_on_time = action_start_logger_.wait_for_timeindex(
										   t + 1, max_inter_action_duration_s_);
	    else
	      {
		action_has_started_on_time = action_start_logger_.wait_for_timeindex(
										   t + 1);
	      }
            if (!action_has_started_on_time)
            {
                std::cout << "Action did not start on time, shutting down. Any "
                             "further actions will be ignored."
                          << std::endl;
                shutdown();
                return;
            }
        }
    }

    static void *loop(void *instance_pointer)
    {
        ((MonitoredRobotDriver *)(instance_pointer))->loop();
        return nullptr;
    }
};

}  // namespace robot_interfaces
