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
    typedef Eigen::Vector4d Quaternion;

    enum JointIndexing {base, center, tip, joint_count};

    Finger() {  }

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

    // \todo: implement forward kinematics
    virtual Vector get_tip_pos() const = 0;

    virtual Vector get_object_pos() const = 0;
    virtual void set_object_pos(const Vector& pos) = 0;

    virtual Quaternion get_object_quat() const = 0;

    virtual Vector get_target_pos() const = 0;
    virtual void set_target_pos(const Vector& pos) const = 0;

    virtual void reset_joints() = 0;

    virtual void wait_for_execution() const = 0;

    Vector get_max_torques() const
    {
        return max_torque_ * Vector::Ones();
    }

    // render frame - only for simulation
    virtual unsigned char* render(std::string mode) = 0;

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

protected:
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




}
