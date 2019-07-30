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

class Minimal
{
public:
    real_time_tools::RealTimeThread thread_;

    Minimal()
    {
        thread_.create_realtime_thread(&Minimal::loop, this);
    }

    static THREAD_FUNCTION_RETURN_TYPE loop(void *instance_pointer)
    {
        ((Minimal *)(instance_pointer))->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }
    void loop()
    {
        while (true)
        {
        }
    }
};

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

    // /**
    //  * @brief this function will
    //  * wait until the previous step is completed,
    //  * store the latest observation in data,
    //  * compute the safe_action based on the desired_action,
    //  * send the safe_action to the robot,
    //  * store both actions in data,
    //  * return data
    //  */
    // virtual Data step(Action desired_action)
    // {
    //     desired_action_->append(desired_action);
    //     TimeIndex t = desired_action_->newest_timeindex();

    //     Data data;
    //     data.desired_action = (*desired_action_)[t];
    //     data.safe_action = (*safe_action_)[t];
    //     data.observation = (*observation_)[t];

    //     return data;
    // }

public: /// we will probably make this private
    virtual Observation get_latest_observation()
    {
        Observation observation;
        observation.angle = get_measured_angles();
        observation.velocity = get_measured_velocities();
        observation.torque = get_measured_torques();
        return observation;
    }
    /// TODO the following three functions should go away and child class should
    /// directly implement the function above.
public:
    virtual Vector get_measured_torques() const = 0;
    virtual Vector get_measured_angles() const = 0;
    virtual Vector get_measured_velocities() const = 0;

protected:
    virtual void apply_action(const Action &action)
    {
        apply_torques(action);
        real_time_tools::Timer::sleep_ms(1.0); /// TODO this is temporary
    }
    /// TODO the following function should go away and child class should
    /// directly implement the function above.
    virtual void apply_torques(const Vector &desired_torques) = 0;

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
    static THREAD_FUNCTION_RETURN_TYPE loop(void *instance_pointer)
    {
        ((NewFinger *)(instance_pointer))->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
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
        double kd = 0.06;
        double max_torque = 0.36;
        Action safe_action = mct::clamp(desired_action,
                                        -max_torque, max_torque);
        safe_action = safe_action - kd * observation.velocity;
        safe_action = mct::clamp(safe_action,
                                 -max_torque, max_torque);
        return safe_action;
    }

    // all the below should go away!!
    virtual Action constrain_action(const Action &desired_action)
    {
        return constrain_torques(desired_action);
    }
    ///TODO: the function below should go away and we should directly
    /// implement the function above.
    virtual Vector constrain_torques(const Vector &desired_torques)
    {
        Vector velocities = get_measured_velocities();
        Vector angles = get_measured_angles();

        Vector constrained_torques;
        for (size_t i = 0; i < desired_torques.size(); i++)
        {
            constrained_torques[i] =
                safety_constraints_[i].get_safe_torque(desired_torques(i),
                                                       velocities(i),
                                                       angles(i));
        }
        return constrained_torques;
    }
    std::array<mct::SafetyConstraint, 3> safety_constraints_;
};

class Finger : public NewFinger
{
public:
    //typedef Eigen::Vector3d Vector;
    typedef Eigen::Vector4d Quaternion; // should go away

    //enum JointIndexing {base, center, tip, joint_count};

    Finger() {}

    virtual void constrain_and_apply_torques(const Vector &desired_torques)
    {
        constrained_torques_ = constrain_torques(desired_torques);
        apply_torques(constrained_torques_);
    }

    virtual Vector get_constrained_torques() const
    {
        return constrained_torques_;
    }

    // \todo: implement forward kinematics
    virtual Vector get_tip_pos() const = 0;

    virtual Vector get_object_pos() const = 0;
    virtual void set_object_pos(const Vector &pos) = 0;

    virtual Quaternion get_object_quat() const = 0;

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

class DisentanglementPlatform
{
public:
    typedef Eigen::Vector3d Vector;

    enum JointIndexing
    {
        table,
        base,
        tip,
        joint_count
    };

    DisentanglementPlatform()
    {
        max_torques_ = 2.0 * 0.02 * 9 * Vector(9.79, 1, 1);

        safety_constraints_[table].min_velocity_ = -0.3;
        safety_constraints_[table].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[table].max_velocity_ = 0.3;
        safety_constraints_[table].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[table].max_torque_ = max_torques_[table];
        safety_constraints_[table].inertia_ = 1.0;      //dummy
        safety_constraints_[table].max_jerk_ = 10000.0; // dummy

        safety_constraints_[base].min_velocity_ = -5.0;
        safety_constraints_[base].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_velocity_ = 5.0;
        safety_constraints_[base].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_torque_ = max_torques_[base];
        safety_constraints_[base].inertia_ = 1.0;      //dummy
        safety_constraints_[base].max_jerk_ = 10000.0; // dummy

        safety_constraints_[tip].min_velocity_ = -5.0;
        safety_constraints_[tip].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_velocity_ = 5.0;
        safety_constraints_[tip].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_torque_ = max_torques_[tip];
        safety_constraints_[tip].inertia_ = 1.0;      //dummy
        safety_constraints_[tip].max_jerk_ = 10000.0; // dummy
    }

    virtual void constrain_and_apply_torques(const Vector &desired_torques)
    {
        constrained_torques_ = constrain_torques(desired_torques);
        apply_torques(constrained_torques_);
    }

    virtual Vector get_constrained_torques() const
    {
        return constrained_torques_;
    }
    virtual Vector get_measured_torques() const = 0;
    virtual Vector get_measured_angles() const = 0;
    virtual Vector get_measured_velocities() const = 0;

    virtual void wait_for_execution() const = 0; /// \todo: this can be implemented here

    Vector get_max_torques() const
    {
        return max_torques_;
    }

protected:
    virtual void apply_torques(const Vector &desired_torques) = 0;

    /// do NOT touch this function unless you know what you are doing, it is
    /// essential for running the robot safely!
    virtual Vector constrain_torques(const Vector &desired_torques)
    {

        Vector velocities = get_measured_velocities();
        Vector angles = get_measured_angles();

        Vector constrained_torques;
        for (size_t i = 0; i < desired_torques.size(); i++)
        {
            constrained_torques[i] =
                safety_constraints_[i].get_safe_torque(desired_torques(i),
                                                       velocities(i),
                                                       angles(i));
        }
        return constrained_torques;
    }

private:
    Vector constrained_torques_;
    std::array<mct::SafetyConstraint, 3> safety_constraints_;
    Vector max_torques_;
};

template <typename Action, typename Observation>
class Robot
{
public:
    struct Data
    {
        Action desired_action;
        Action safe_action;
        Observation observation;
    };

    /**
     * @brief this function will 
     * wait until the previous step is completed,
     * store the latest observation in data,
     * compute the safe_action based on the desired_action,
     * send the safe_action to the robot,
     * store both actions in data,
     * return data
     */
    virtual Data step(Action desired_action) = 0;
};

} // namespace robot_interfaces
