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
#include <cstdint>

#include <Eigen/Eigen>

#include <mpi_cpp_tools/basic_tools.hpp>
#include <mpi_cpp_tools/dynamical_systems.hpp>
#include <mpi_cpp_tools/math.hpp>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

/**
 * @brief Here we define all the necessary classes
 * for interaction with the robot. The main classes are:
 * - RobotDriver: takes care of getting observations and
 *          applying actions on the robot. Also makes
 *          sure that timing is satisfied and shuts down
 *          otherwise.
 *
 * - RobotData: A class containing all the inputs (actions)
 *              and outputs (observations) of the robot.
 *              It stores a histories of inputs and outputs,
 *              which are synchronized.
 *
 * - RobotBackend: Takes care of communication between RobotDriver
 *                and RobotData. It applies actions and
 *                requests observations from the robot at
 *                the appropriate times.
 *
 * - RobotFrontend: A wrapper around RobotData facilitating its
 *                usage for the end-user.
 *
 */

namespace robot_interfaces
{

/**
 * @brief Driver for interfacing the actual robot hardware or simulation.
 *
 * Interface to the robot used by the subsequent classes. Any robot (be it real
 * or simulation) has to derive from this class and implement the functions
 * apply_action(), get_latest_observation() and shutdown().
 * This Base class provides some timing logic around those three functions. It
 * makes sure that after the first call of apply_action(), it is always called
 * again after some specified time, otherwise the shutdown() method will
 * be called. This Base class also makes sure that the apply_action() function
 * itself does not take more time than expected.
 *
 * @tparam Action
 * @tparam Observation
 */
template <typename Action, typename Observation>
class RobotDriver
{
public:
    RobotDriver(const double &max_action_duration_s,
          const double &max_inter_action_duration_s)
        : max_action_duration_s_(max_action_duration_s),
          max_inter_action_duration_s_(max_inter_action_duration_s),
          is_shutdown_(false),
          action_start_logger_(1000),
          action_end_logger_(1000)
    {
        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
        thread_->create_realtime_thread(&RobotDriver::loop, this);
    }

    ~RobotDriver()
    {
        shutdown_and_stop_thread();
        thread_->join();
    }

    double get_max_inter_action_duration_s()
    {
        return max_inter_action_duration_s_;
    }

    /**
     * @brief Apply desired action on the robot while making sure timing is
     *        respected.
     *
     * Concretely, it makes sure that the execution of an action does not take
     * more than max_action_duration_s_ seconds and that the time interval
     * between the termination of the previous action and the receival (through
     * apply_action()) of the next action will not exceed
     * max_inter_action_duration_s_ seconds. If these timing constraints are not
     * satisfied, the robot will be shutdown, and no more actions from the
     * outside will be accepted.
     *
     * @param desired_action  The desired action.
     * @return  The action that is actually applied on the robot (may differ
     *     from desired action due to safety limitations).
     */
    virtual Action apply_action_and_check_timing(
        const Action &desired_action) final
    {
        if (is_shutdown_)
        {
            // FIXME I don't think it makes sense to return the desired action
            // in case of shutdown.  Shouldn't it rather be s.th. like Zero()?
            return desired_action;
        }
        action_start_logger_.append(true);
        Action applied_action = apply_action(desired_action);
        action_end_logger_.append(true);
        return applied_action;
    }

protected:
    /**
     * @brief Apply action immediately and block until it is executed.
     *
     * This method must apply the desired_action immediately when it is called,
     * and only return once the action has been executed completely.  This way
     * we can accommodate both simulators and real robots with this interface.
     *
     * @param desired_action  The action we want to apply.
     * @return  The action that was actually applied (since due to safety
     *     reasons it might not be possible to apply the desired action).
     */
    virtual Action apply_action(const Action &desired_action) = 0;

public:
    /**
     * @brief Return the latest observation immediately.
     *
     * @return Observation
     */
    virtual Observation get_latest_observation() = 0;

protected:

    /**
     * @brief Shut down the robot safely.
     *
     * After shutdown, actions send by the user are ignored.
     */
    virtual void shutdown_and_stop_thread() final
    {
        if (!is_shutdown_)
        {
            is_shutdown_ = true;
            shutdown();
        }
    }

    /**
     * @brief Shut down the robot safely.
     *
     */
    virtual void shutdown() = 0;

private:
    void loop()
    {
        real_time_tools::set_cpu_dma_latency(0);

        while (!is_shutdown_ &&
               !action_start_logger_.wait_for_timeindex(0, 0.1))
        {
        }

        for (size_t t = 0; !is_shutdown_; t++)
        {
            bool action_has_ended_on_time =
                action_end_logger_.wait_for_timeindex(t,
                                                      max_action_duration_s_);
            if (!action_has_ended_on_time)
            {
                std::cout
                    << "action did not end on time, shutting down. any further "
                       "actions will be ignored."
                    << std::endl;
                shutdown_and_stop_thread();
                return;
            }

            bool action_has_started_on_time =
                action_start_logger_.wait_for_timeindex(
                    t + 1, max_inter_action_duration_s_);
            if (!action_has_started_on_time)
            {
                std::cout << "action did not start on time, shutting down. any "
                             "further "
                             "actions will be ignored."
                          << std::endl;
                shutdown_and_stop_thread();
                return;
            }
        }
    }
    static void *loop(void *instance_pointer)
    {
        ((RobotDriver *)(instance_pointer))->loop();
        return nullptr;
    }

private:
    double max_action_duration_s_;
    double max_inter_action_duration_s_;

    bool is_shutdown_;  // TODO: should be atomic

    real_time_tools::ThreadsafeTimeseries<bool> action_start_logger_;
    real_time_tools::ThreadsafeTimeseries<bool> action_end_logger_;

    std::shared_ptr<real_time_tools::RealTimeThread> thread_;
};


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
    using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;
    // TODO this is not quite clean because we should not have to specify a type
    // here.
    typedef Timeseries<int>::Index TimeIndex;
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;

    RobotData(size_t history_length = 1000, bool use_shared_memory = false,
              std::string shared_memory_address = "")
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

/**
 * @brief Communication link between RobotDriver and RobotData.
 *
 * At each time-step, it gets the observation from the RobotDriver and writes it
 * to RobotData, and it takes the desired_action from RobotData and applies it
 * on the RobotDriver.
 *
 * @tparam Action
 * @tparam Observation
 */
template <typename Action, typename Observation>
class RobotBackend
{
public:
    struct Status
    {
        uint32_t action_repetitions;
    };

    // TODO add parameter: n_max_repeat_of_same_action
    RobotBackend(std::shared_ptr<RobotDriver<Action, Observation>> robot,
                std::shared_ptr<RobotData<Action, Observation, Status>> robot_data)
        : robot_(robot),
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

    int get_max_action_repetitions() { return max_action_repetitions_; }

    void set_max_action_repetitions(const int &max_action_repetitions)
    {
        max_action_repetitions_ = max_action_repetitions;
    }

private:
    std::shared_ptr<RobotDriver<Action, Observation>> robot_;
    std::shared_ptr<RobotData<Action, Observation, Status>> robot_data_;
    bool destructor_was_called_;  // should be atomic
    int max_action_repetitions_;

    std::vector<real_time_tools::Timer> timers_;

    std::shared_ptr<real_time_tools::RealTimeThread> thread_;

    // control loop ------------------------------------------------------------
    static void *loop(void *instance_pointer)
    {
        ((RobotBackend *)(instance_pointer))->loop();
        return nullptr;
    }

    /**
     * @brief Main loop.
     *
     * It will essentially iterate over robot_data_.desired_action and apply
     * these actions to the robot, and it will read the applied_action and the
     * observation from the robot and append them to the corresponding
     * timeseries in robot_data_.
     *
     */
    void loop()
    {
        timers_.resize(10);
        real_time_tools::set_cpu_dma_latency(0);

        // wait until first desired_action was received
        // ----------------------------
        while (!destructor_was_called_ &&
               !robot_data_->desired_action->wait_for_timeindex(0, 0.1))
        {
        }

        for (long int t = 0; !destructor_was_called_; t++)
        {
            // TODO: figure out latency stuff!! open /dev/cpu_dma_latency:
            // Permission denied

            timers_[0].tac_tic();

            timers_[6].tic();
            // get latest observation from robot and append it to robot_data_
            // --------
            Observation observation = robot_->get_latest_observation();
            timers_[6].tac();

            timers_[1].tic();
            robot_data_->observation->append(observation);
            // TODO: for some reason this sometimes takes more than 2 ms
            // i think this may be due to a non-realtime thread blocking the
            // timeseries. this is in fact an issue, we might have to duplicate
            // all the timeseries and have a realtime thread writing back and
            // forth
            timers_[1].tac();

            timers_[2].tic();
            // if the robot has a finite max_inter_action_duration_s (not NAN or
            // infinite, meaning it requires receiving actions in fixed time
            // intervals), but robot_data_ has not received yet the next action
            // to apply, we optionally repeat the previous action.
            Status status = {0};
            if (std::isfinite(robot_->get_max_inter_action_duration_s()) &&
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
            }
            robot_data_->status->append(status);
            timers_[2].tac();

            timers_[3].tic();
            // TODO: this may wait forever
            Action desired_action = (*robot_data_->desired_action)[t];
            timers_[3].tac();
            timers_[4].tic();
            Action applied_action =
                robot_->apply_action_and_check_timing(desired_action);
            timers_[4].tac();
            timers_[5].tic();
            robot_data_->applied_action->append(applied_action);
            timers_[5].tac();

            // if (t % 5000 == 0) {
            //   for (size_t i = 0; i < 7; i++) {
            //     std::cout << i << " --------------------------------------"
            //               << std::endl;
            //     timers_[i].print_statistics();
            //   }
            // }
        }
    }
};

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
    template <typename Type>
    using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;
    typedef Timeseries<int>::Index TimeIndex;
    typedef Timeseries<int>::Timestamp TimeStamp;

    typedef typename RobotBackend<Action, Observation>::Status Status;

    RobotFrontend(std::shared_ptr<RobotData<Action, Observation, Status>> robot_data)
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

namespace finger
{

// Typedefs for all the templated types to be used for the Finger robot.

typedef Eigen::Vector3d Vector;

typedef Vector Action;
struct Observation
{
    Vector angle;
    Vector velocity;
    Vector torque;
};

template <typename Type>
using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;

typedef Timeseries<int>::Index TimeIndex;

typedef RobotBackend<Action, Observation> Backend;
typedef std::shared_ptr<Backend> BackendPtr;
typedef Backend::Status Status;

typedef RobotData<Action, Observation, Status> Data;
typedef std::shared_ptr<Data> DataPtr;

//! @brief Frontend for the Finger robot.
typedef RobotFrontend<Action, Observation> Frontend;
typedef std::shared_ptr<Frontend> FrontendPtr;

}  // namespace finger


}  // namespace robot_interfaces
