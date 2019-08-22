///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////


#pragma once

#include <cmath>
#include <math.h>
#include <Eigen/Eigen>
#include <algorithm>

#include "mpi_cpp_tools/basic_tools.hpp"
#include "mpi_cpp_tools/math.hpp"
#include "mpi_cpp_tools/dynamical_systems.hpp"

#include "real_time_tools/threadsafe/threadsafe_timeseries.hpp"
#include "real_time_tools/timer.hpp"
#include "real_time_tools/thread.hpp"

namespace robot_interfaces
{

// |------ t = 0 ------|------ t = 1 ------|
// |----- action0 -----|----- action1 -----|
// o                   o                   o
// b                   b                   b
// s                   s                   s
// 0                   1                   2

// action_1 -> action_2 ...
//         \   A
//          \ /
//           X
//          / \
//         /   V
// observ_1 -> observ_2 ...
class NewFinger
{
public:
    enum JointIndexing
    {
        base,
        center,
        tip,
        joint_count
    };

    template <typename Type>
    using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;

    typedef Timeseries<int>::Index TimeIndex; // \TODO this is not quite clean because we should not have to specify a type here.
    typedef Timeseries<int>::Timestamp TimeStamp;

    typedef Eigen::Vector3d Vector;

    typedef Eigen::Vector3d Action;

    struct Observation
    {
        Vector angle;
        Vector velocity;
        Vector torque;
    };

    /// TODO: remove default values!!!
    NewFinger(const double &expected_step_duration_ms = 1.0,
              const double &step_duration_tolerance_ratio = 2.0,
              const bool &is_realtime = true) : expected_step_duration_ms_(expected_step_duration_ms),
                                                step_duration_tolerance_ratio_(step_duration_tolerance_ratio),
                                                is_realtime_(is_realtime),
                                                is_paused_(false)
    {

        size_t history_length = 1000;
        desired_action_ = std::make_shared<Timeseries<Action>>(history_length);
        safe_action_ = std::make_shared<Timeseries<Action>>(history_length);
        observation_ = std::make_shared<Timeseries<Observation>>(history_length);

        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
        thread_->create_realtime_thread(&NewFinger::loop, this);
    }

    Observation get_observation(const TimeIndex &t)
    {
        return (*observation_)[t];
    }
    Action get_desired_action(const TimeIndex &t)
    {
        return (*desired_action_)[t];
    }
    Action get_safe_action(const TimeIndex &t)
    {
        return (*safe_action_)[t];
    }
    TimeStamp get_time_stamp_ms(const TimeIndex &t)
    {
        return observation_->timestamp_ms(t);
    }

    TimeIndex append_desired_action(const Action &desired_action)
    {
        // TODO: we should make sure not so many actions are appended
        // that we do not have enough history containing all actions to
        // be applied.
        is_paused_ = false;
        desired_action_->append(desired_action);
        return desired_action_->newest_timeindex();
    }

    void wait_until_time_index(const TimeIndex &t)
    {
        observation_->timestamp_ms(t);
    }
    TimeIndex current_time_index()
    {
        return observation_->newest_timeindex();
    }
    void pause()
    {
        is_paused_ = true;
        desired_action_->append(Action::Zero());
    }

protected:
    virtual Observation get_latest_observation() = 0;

protected:
    virtual void apply_action(const Action &action) = 0;

private:
    std::shared_ptr<Timeseries<Action>> desired_action_;
    std::shared_ptr<Timeseries<Action>> safe_action_;
    std::shared_ptr<Timeseries<Observation>> observation_;
    std::shared_ptr<real_time_tools::RealTimeThread> thread_;

    bool is_paused_;

    double expected_step_duration_ms_;
    double step_duration_tolerance_ratio_;
    bool is_realtime_;

    // control loop ------------------------------------------------------------
    static void* loop(void *instance_pointer)
    {
        ((NewFinger *)(instance_pointer))->loop();
        return nullptr;
    }
    void loop()
    {
        // TODO: there is a slight problem here: this thread
        // may start running before child class is created, and
        // hence attempt to call nonexistant function.
        for (TimeIndex t = 0; true; t++)
        {
            if (is_realtime_ && is_paused_ &&
                (desired_action_->length() == 0 ||
                 desired_action_->newest_timeindex() < t))
            {
                desired_action_->append(Action::Zero());
            }

            Action desired_action = (*desired_action_)[t];
            Observation observation = get_latest_observation();
            Action safe_action = compute_safe_action(desired_action,
                                                     observation);
            safe_action_->append(safe_action);
            observation_->append(observation);

            // safe_action_->append(constrain_action();

            apply_action((*safe_action_)[t]);

            if (t >= 1)
            {
                check_timing(observation_->timestamp_ms(t) -
                             observation_->timestamp_ms(t - 1));
            }
        }

        // TODO: we have to make sure this loop exits properly at destruction time!!
    }

    // helper functions --------------------------------------------------------
    void check_timing(const double &actual_step_duration_ms)
    {
        double max_step_duration_ms =
            expected_step_duration_ms_ * step_duration_tolerance_ratio_;
        double min_step_duration_ms =
            expected_step_duration_ms_ / step_duration_tolerance_ratio_;

        if (is_realtime_ &&
            (actual_step_duration_ms > max_step_duration_ms ||
             actual_step_duration_ms < min_step_duration_ms))
        {
            std::ostringstream oss;
            oss << "control loop did not run at expected rate "
                   "did you provide actions fast enough? "
                << std::endl
                << "expected step duration: "
                << expected_step_duration_ms_ << std::endl
                << "actual step duration: "
                << actual_step_duration_ms << std::endl
                << "tolearance ratio: "
                << step_duration_tolerance_ratio_ << std::endl;
            throw std::runtime_error(oss.str());
        }
    }

protected:
    virtual Action compute_safe_action(const Action &desired_action,
                                       const Observation &observation)
    {
        // Vector kd(0.04, 0.08, 0.02);

        Vector kd(0.08, 0.08, 0.04);
        double max_torque = 0.36;
        Action safe_action = mct::clamp(desired_action,
                                        -max_torque, max_torque);
        safe_action = safe_action - kd.cwiseProduct(observation.velocity);
        safe_action = mct::clamp(safe_action,
                                 -max_torque, max_torque);
        return safe_action;
    }
};

class Finger : public NewFinger
{
public:
    //typedef Eigen::Vector3d Vector;
    typedef Eigen::Vector4d Quaternion; // should go away

    //enum JointIndexing {base, center, tip, joint_count};

    Finger() {}

    virtual Vector get_constrained_torques() const
    {
        return constrained_torques_;
    }

    // \todo: implement forward kinematics
    virtual Vector get_tip_pos() const = 0;

    virtual Vector get_object_pos(std::string object_name) const = 0;
    virtual void set_object_pos(const Vector &pos, std::string object_name) = 0;

    virtual Quaternion get_object_quat(std::string object_name) const = 0;

    virtual Vector get_target_pos() const = 0;
    virtual void set_target_pos(const Vector &pos) const = 0;

    virtual void reset_joints() = 0;

    virtual void wait_for_execution() const = 0;

    Vector get_max_torques() const
    {
        return max_torque_ * Vector::Ones();
    }

    // render frame - only for simulation
    virtual unsigned char *render(std::string mode) = 0;

protected:
protected:
    Vector constrained_torques_;
    double max_torque_;
};


} // namespace robot_interfaces
