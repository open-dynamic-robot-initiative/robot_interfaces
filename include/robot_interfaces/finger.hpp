#pragma once

#include <Eigen/Eigen>

namespace robot_interfaces
{

class Finger
{
public:
    typedef Eigen::Vector3d Vector;

    enum JointIndexing {base, center, tip, joint_count};


    Finger()
    {
        max_torques_ = Vector::Ones() * 2.0 * 0.02 * 9.0;
        max_velocities_ = Vector::Ones() * 2.0;
        min_angles_ = std::numeric_limits<double>::quiet_NaN()*Vector::Ones();
        max_angles_ = std::numeric_limits<double>::quiet_NaN()*Vector::Ones();

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

protected:
    virtual void apply_torques(const Vector& desired_torques) = 0;

    /// do NOT touch this function unless you know what you are doing, it is
    /// essential for running the robot safely!
    virtual Vector constrain_torques(const Vector& desired_torques)
    {
        Vector constrained_torques;

        for(size_t i = 0; i < desired_torques.size(); i++)
        {
            double torque = desired_torques(i);

            torque = std::min(torque, max_torques_(i));
            torque = std::max(torque, -max_torques_(i));

            // Velocity safety feature.
            if (!std::isnan(max_velocities_(i)) && std::fabs(
                        get_measured_velocities()(i)) > max_velocities_(i))
                torque = 0;

            // Joint limits safety feature.
            if (!std::isnan(max_angles_(i))
                    && get_measured_angles()(i) > max_angles_(i))
                torque = -max_torques_(i);
            if (!std::isnan(min_angles_(i))
                    && get_measured_angles()(i) < min_angles_(i))
                torque = max_torques_(i);

            constrained_torques(i) = torque;
        }

        return constrained_torques;
    }

    void set_max_torques(const Vector& max_torques)
    {
        for(size_t i = 0; i < max_torques.size(); i++)
        {
            if (max_torques(i) < 0)
                throw std::invalid_argument("the current limit must be "
                                            "a positive number.");
        }
        max_torques_ = max_torques;
    }
    void set_max_velocities(const Vector& max_velocities)
    {
        for(size_t i = 0; i < max_velocities.size(); i++)
        {
            if (!std::isnan(max_velocities(i)) && max_velocities(i) < 0)
                throw std::invalid_argument("the velocity limit must be "
                                            "a positive number or NaN.");
        }

        max_velocities_ = max_velocities;
    }
    void set_angle_limits(const Vector& min_angles,
                          const Vector& max_angles)
    {
        for(size_t i = 0; i < min_angles.size(); i++)
        {
            if (!std::isnan(min_angles(i)) && !std::isnan(max_angles(i)) && (
                        min_angles(i) > max_angles(i) ||
                        max_angles(i) - min_angles(i) > 2*M_PI))
                throw std::invalid_argument("Invalid joint limits. Make sure "
                                            "the interval denoted by the joint "
                                            "limits is a valid one.");
        }
        min_angles_ = min_angles;
        max_angles_ = max_angles;
    }


    Vector get_max_torques() const
    {
        return max_torques_;
    }
    Vector get_max_velocities_() const
    {
        return max_velocities_;
    }
    Vector get_min_angles_() const
    {
        return min_angles_;
    }
    Vector get_max_angles_() const
    {
        return max_angles_;
    }



private:
    Vector constrained_torques_;

    Vector max_torques_;
    Vector max_velocities_;
    Vector min_angles_;
    Vector max_angles_;
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
