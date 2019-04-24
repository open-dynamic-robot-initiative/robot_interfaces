#pragma once

#include <cmath>
#include <math.h>
#include <Eigen/Eigen>
#include <algorithm>

#include "mpi_cpp_tools/basic_tools.hpp"
#include "mpi_cpp_tools/math.hpp"
#include "mpi_cpp_tools/dynamical_systems.hpp"




namespace robot_interfaces
{


class Finger
{
public:
    typedef Eigen::Vector3d Vector;

    enum JointIndexing {base, center, tip, joint_count};


    Finger()
    {
        max_torque_ = 2.0 * 0.02 * 9.0;

        safety_constraints_[base].min_velocity_ = -6.0;
        safety_constraints_[base].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_velocity_ = 6.0;
        safety_constraints_[base].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_torque_ = max_torque_;
        safety_constraints_[base].inertia_ = 0.004;
        safety_constraints_[base].max_jerk_ = 2 * max_torque_ / safety_constraints_[base].inertia_ / 0.05;


        safety_constraints_[center].min_velocity_ = -6.0;
        safety_constraints_[center].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[center].max_velocity_ = 6.0;
        safety_constraints_[center].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[center].max_torque_ = max_torque_;
        safety_constraints_[center].inertia_ = 0.004;
        safety_constraints_[center].max_jerk_ = 2 * max_torque_ / safety_constraints_[center].inertia_ / 0.05;


        safety_constraints_[tip].min_velocity_ = -20.0;
        safety_constraints_[tip].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_velocity_ = 20.0;
        safety_constraints_[tip].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_torque_ = max_torque_;
        safety_constraints_[tip].inertia_ = 0.001;
        safety_constraints_[tip].max_jerk_ = 2 * max_torque_ / safety_constraints_[tip].inertia_ / 0.05;
    }

    virtual void constrain_and_apply_torques(const Vector& desired_torques)
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

    virtual void wait_for_execution() const = 0;

    Vector get_max_torques() const
    {
        return max_torque_ * Vector::Ones();
    }

protected:
    virtual void apply_torques(const Vector& desired_torques) = 0;

    virtual Vector constrain_torques(const Vector& desired_torques)
    {
        Vector velocities = get_measured_velocities();
        Vector angles = get_measured_angles();

        Vector constrained_torques;
        for(size_t i = 0; i < desired_torques.size(); i++)
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
    double max_torque_;
};



class DisentanglementPlatform
{
public:
    typedef Eigen::Vector3d Vector;

    enum JointIndexing {table, base, tip, joint_count};


    DisentanglementPlatform()
    {
        max_torques_ = 2.0 * 0.02 * 9 * Vector(9.79, 1, 1);

        safety_constraints_[table].min_velocity_ = -0.3;
        safety_constraints_[table].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[table].max_velocity_ = 0.3;
        safety_constraints_[table].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[table].max_torque_ = max_torques_[table];
        safety_constraints_[table].inertia_ = 1.0; //dummy
        safety_constraints_[table].max_jerk_ = 10000.0; // dummy

        safety_constraints_[base].min_velocity_ = -5.0;
        safety_constraints_[base].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_velocity_ = 5.0;
        safety_constraints_[base].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_torque_ = max_torques_[base];
        safety_constraints_[base].inertia_ = 1.0; //dummy
        safety_constraints_[base].max_jerk_ =  10000.0; // dummy


        safety_constraints_[tip].min_velocity_ = -5.0;
        safety_constraints_[tip].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_velocity_ = 5.0;
        safety_constraints_[tip].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_torque_ = max_torques_[tip];
        safety_constraints_[tip].inertia_ = 1.0; //dummy
        safety_constraints_[tip].max_jerk_ = 10000.0; // dummy
    }

    virtual void constrain_and_apply_torques(const Vector& desired_torques)
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
    virtual void apply_torques(const Vector& desired_torques) = 0;

    /// do NOT touch this function unless you know what you are doing, it is
    /// essential for running the robot safely!
    virtual Vector constrain_torques(const Vector& desired_torques)
    {

        Vector velocities = get_measured_velocities();
        Vector angles = get_measured_angles();

        Vector constrained_torques;
        for(size_t i = 0; i < desired_torques.size(); i++)
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








template<typename Action, typename Observation> class Robot
{
public:

    /**
     * @brief this function will execute the action. in real-time mode this
     * means simply that the action will be sent to the system. in
     * non-real-time mode this means that the action will be simulated for
     * one time step and then the simulation will be stopped.
     *
     * @param action to apply
     */
    virtual void constrain_and_apply_action(const Action& action) = 0;

    /**
     * @brief this function only returns once the current action has been
     * completely exectued. in real-time mode this means it will wait until
     * one time step has elapsed since execute has been called.
     * in non-real-time mode this means that it will wait until the simulator
     * has ran the action for the lenth of one time step.
     */
    virtual void wait_for_execution() const = 0;

    virtual Observation get_observation() const = 0;

};




}
