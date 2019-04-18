#pragma once

#include <cmath>
#include <math.h>
#include <Eigen/Eigen>
#include <algorithm>



namespace robot_interfaces
{


class NonnegDouble
{
public:
    NonnegDouble()
    {
        value_ = 0;
    }


    NonnegDouble(double value)
    {
        if(!isnan(value) && value < 0.0)
            throw std::invalid_argument("expected nonnegative double");
        value_ = value;
    }

    operator double() const
    {
        return value_;
    }

private:
    double value_;
};


struct Constraint
{
    bool is_satisfied(const double& angle, const double& velocity)
    {
        if(angle < min_angle && velocity < min_velocity)
            return false;

        if(angle > max_angle && velocity > max_velocity)
            return false;

        return true;
    }

    double min_angle;
    double min_velocity;
    double max_angle;
    double max_velocity;
};







class Finger
{
public:
    typedef Eigen::Vector3d Vector;

    enum JointIndexing {base, center, tip, joint_count};


    Finger()
    {
        max_torques_ = Vector::Ones() * 2.0 * 0.02 * 9.0;
        max_velocities_ = Vector::Ones() * 2.0;
        inertias_ = Vector(0.004, 0.004, 0.001);

        state_constraints_[base].min_angle = std::numeric_limits<double>::infinity();
        state_constraints_[base].max_angle = -std::numeric_limits<double>::infinity();
        state_constraints_[base].min_velocity = -6.0;
        state_constraints_[base].max_velocity = 6.0;

        state_constraints_[center].min_angle = std::numeric_limits<double>::infinity();
        state_constraints_[center].max_angle = -std::numeric_limits<double>::infinity();
        state_constraints_[center].min_velocity = -6.0;
        state_constraints_[center].max_velocity = 6.0;

        state_constraints_[tip].min_angle = std::numeric_limits<double>::infinity();
        state_constraints_[tip].max_angle = -std::numeric_limits<double>::infinity();
        state_constraints_[tip].min_velocity = -20.0;
        state_constraints_[tip].max_velocity = 20.0;
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
    Vector get_max_velocities_() const
    {
        return max_velocities_;
    }
    //    Vector get_min_angles_() const
    //    {
    //        return min_angles_;
    //    }
    //    Vector get_max_angles_() const
    //    {
    //        return max_angles_;
    //    }



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

            // torque limit ----------------------------------------------------
            torque = clamp(torque, -max_torques_(i), max_torques_(i));

            // velocity limit --------------------------------------------------
            if (!std::isnan(max_velocities_(i)) && std::fabs(
                        get_measured_velocities()(i)) > max_velocities_(i))
                torque = 0;

            // angle limit -----------------------------------------------------
            //            double angle = get_measured_angles()(i);
            //            double velocity = get_measured_velocities()(i);

            //            if (!std::isnan(max_angles_(i)) &&
            //                    velocity > max_collision_velocities_(i))
            //            {
            //                double distance_to_braking_zone =
            //                        compute_distance_to_braking_zone(
            //                            velocity,
            //                            max_angles_(i) - angle,
            //                            max_torques_(i),
            //                            inertias_(i),
            //                            max_collision_velocities_(i));

            //                double safety_weight =
            //                        (delta - distance_to_braking_zone) / delta;
            //                safety_weight = clamp(safety_weight, 0, 1);

            //                torque = (1.0 - safety_weight) * torque
            //                        + safety_weight * (-max_torques_(i));

            //            }


            //            if (!std::isnan(max_angles_(i)) &&
            //                    velocity < -max_collision_velocities_(i))
            //            {
            //                double distance_to_braking_zone =
            //                        compute_distance_to_braking_zone(
            //                            -velocity,
            //                            max_angles_(i) - angle,
            //                            max_torques_(i),
            //                            inertias_(i),
            //                            max_collision_velocities_(i));

            //                double safety_weight =
            //                        (delta - distance_to_braking_zone) / delta;
            //                safety_weight = clamp(safety_weight, 0, 1);

            //                torque = (1.0 - safety_weight) * torque
            //                        + safety_weight * (-max_torques_(i));

            //            }





            //            double velocity = get_measured_velocities()(i);
            //            double admissible_collision_velocity = max_collision_velocities_(i);
            //            double max_deceleration = max_torques_(i) / inertias_(i);

            //            double deceleration_distance =
            //                    (std::fabs(velocity) - admissible_collision_velocity) *
            //                    (std::fabs(velocity) + admissible_collision_velocity) /
            //                    (2 * max_deceleration);



            //            double distance_to_max_angle =
            //                    max_angles_(i) - get_measured_angles()(i);
            //            double distance_to_min_angle =
            //                    get_measured_angles()(i) - min_angles_(i);


            //            double delta = 0.00000001;


            //            double distance_to_braking_zone =
            //                    distance_to_max_angle - deceleration_distance;
            //            if (!std::isnan(max_angles_(i)) &&
            //                    velocity > admissible_collision_velocity
            //                    && distance_to_braking_zone < delta)
            //            {


            //                double safety_weight =
            //                        (delta - distance_to_braking_zone) / delta;
            //                safety_weight = clamp(safety_weight, 0, 1);

            //                torque = (1.0 - safety_weight) * torque
            //                        + safety_weight * (-max_torques_(i));




            //                std::cout << "upper limit" << std::endl;
            //                std::cout << "velocity: " << velocity << std::endl;
            //                std::cout << "admissible_collision_velocity: " << admissible_collision_velocity << std::endl;
            //                std::cout << "deceleration_distance: " << deceleration_distance << std::endl;
            //                std::cout << "distance_to_max_angle: " << distance_to_max_angle << std::endl;



            //                //                torque = -max_torques_(i);
            //            }
            //            if (!std::isnan(min_angles_(i)) &&
            //                    velocity < -admissible_collision_velocity &&
            //                    deceleration_distance > distance_to_min_angle)
            //            {
            //                std::cout << "lower limit" << std::endl;
            //                std::cout << "velocity: " << velocity << std::endl;
            //                std::cout << "admissible_collision_velocity: " << admissible_collision_velocity << std::endl;
            //                std::cout << "deceleration_distance: " << deceleration_distance << std::endl;
            //                std::cout << "distance_to_min_angle: " << distance_to_min_angle << std::endl;


            //                torque = max_torques_(i);
            //            }

            //            constrained_torques(i) = torque;
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



        for(size_t i = 0; i < state_constraints_.size(); i++)
        {
            state_constraints_[i].min_angle = min_angles[i];
            state_constraints_[i].max_angle = max_angles[i];
        }
    }

private:
    static double clamp(const double& value, const double& lo, const double& hi)
    {
        return std::max(lo, std::min(value, hi));
    }

    //    static double compute_acceleration_distance(
    //            const double& initial_velocity,
    //            const double& target_velocity,
    //            const double& force,
    //            const NonnegDouble& inertia)
    //    {
    //        if (std::signbit(target_velocity - initial_velocity) !=
    //                std::signbit(force))
    //        {
    //            return std::numeric_limits<double>::quiet_NaN();
    //        }

    //        double acceleration = force / inertia;

    //        double acceleration_distance =
    //                (initial_velocity - target_velocity) *
    //                (initial_velocity + target_velocity) /
    //                (2 * acceleration);

    //        return acceleration_distance;
    //    }





    //    static double compute_braking_weight(
    //            const double& position,
    //            const double& velocity,
    //            const double& wall_position,
    //            )
    //    {
    //        if (max_collision_velocity > velocity)
    //            throw std::invalid_argument("we expect "
    //                                        "max_collision_velocity <= velocity.");

    //        double max_deceleration = max_braking_force / inertia;

    //        double deceleration_distance =
    //                (velocity - max_collision_velocity) *
    //                (velocity + max_collision_velocity) /
    //                (2 * max_deceleration);

    //        return deceleration_distance;
    //    }






private:
    Vector constrained_torques_;

    Vector max_torques_;
    Vector inertias_;

    Vector max_velocities_; /// \todo: this should go away, it should be absorbed by the state constraints

    std::array<Constraint, 3> state_constraints_;
};

template<typename Vector>
void append_to_vector(Vector& vector,
                      const double& element)
{
    vector.conservativeResize(vector.size() + 1);
    vector[vector.size() - 1] = element;
}


class LinearDynamics
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 3> Vector;

    LinearDynamics(Eigen::Vector4d parameters): LinearDynamics(parameters[0],
                                                parameters[1],
                                                parameters[2],
                                                parameters[3]) { }


    LinearDynamics(double jerk,
                   double initial_acceleration,
                   double initial_velocity,
                   double initial_position)
    {
        jerk_ = jerk;
        initial_acceleration_ = initial_acceleration;
        initial_velocity_ = initial_velocity;
        initial_position_= initial_position;
    }
    double get_acceleration(NonnegDouble t) const
    {
        return  jerk_ * t +
                initial_acceleration_;
    }
    double get_velocity(NonnegDouble t) const
    {
        return jerk_ * 0.5 * pow(t,2) +
                initial_acceleration_ * t +
                initial_velocity_;
    }
    double get_position(NonnegDouble t) const
    {
        return jerk_ * 0.5 * 1./3. * pow(t, 3) +
                initial_acceleration_ * 0.5 * pow(t, 2) +
                initial_velocity_ * t +
                initial_position_;
    }

    Vector find_t_given_velocity(double velocity) const
    {
        double a = jerk_ * 0.5;
        double b = initial_acceleration_;
        double c = initial_velocity_ - velocity;

        double determinant = pow(b, 2) - 4 * a * c;

        Vector solutions(0);
        if(a == 0)
        {
            if(b != 0)
            {
                solutions.resize(1);
                solutions[0] = - c / b;
            }

        }
        else if(determinant == 0)
        {
            solutions.resize(1);
            solutions[0] = -b / 2 / a;
        }
        else if(determinant > 0)
        {
            double determinant_sqrt = std::sqrt(determinant);
            solutions.resize(2);
            solutions[0] = (-b + determinant_sqrt) / 2 / a;
            solutions[1] = (-b - determinant_sqrt) / 2 / a;
        }

        Vector positive_solutions(0);
        for(size_t i = 0; i < solutions.size(); i++)
        {
            if(solutions[i] >= 0)
            {
                append_to_vector(positive_solutions, solutions[i]);
            }
        }

        return positive_solutions;
    }

protected:
    double jerk_;
    double initial_acceleration_;
    double initial_velocity_;
    double initial_position_;
};





class LinearDynamicsWithAccelerationConstraint: public LinearDynamics
{
public:
    typedef LinearDynamics::Vector Vector;

    LinearDynamicsWithAccelerationConstraint(Eigen::Matrix<double, 5, 1> parameters):
        LinearDynamicsWithAccelerationConstraint(parameters[0],
        parameters[1],
        parameters[2],
        parameters[3],
        parameters[4]) { }

    LinearDynamicsWithAccelerationConstraint(double jerk,
                                             double initial_acceleration,
                                             double initial_velocity,
                                             double initial_position,
                                             NonnegDouble abs_acceleration_limit):
        LinearDynamics(jerk,
                       initial_acceleration,
                       initial_velocity,
                       initial_position)
    {
        if(jerk_ > 0)
            acceleration_limit_ = abs_acceleration_limit;
        else
            acceleration_limit_ = -abs_acceleration_limit;

        jerk_duration_ =
                (acceleration_limit_ - initial_acceleration) / jerk;

        set_initial_acceleration(initial_acceleration);
    }


    void set_initial_acceleration(double initial_acceleration)
    {
        if(std::fabs(initial_acceleration) > std::fabs(acceleration_limit_))
            throw std::invalid_argument("expected "
                                        "std::fabs(initial_acceleration) > "
                                        "abs_acceleration_limit");
    }

    double get_acceleration(NonnegDouble t) const
    {
        if(t < jerk_duration_)
        {
            return  LinearDynamics::get_acceleration(t);
        }
        else
        {
            return acceleration_limit_;
        }
    }
    double get_velocity(NonnegDouble t) const
    {
        if(t < jerk_duration_)
        {
            return LinearDynamics::get_velocity(t);
        }
        else
        {
            return LinearDynamics::get_velocity(jerk_duration_) +
                    acceleration_limit_ * (t - jerk_duration_);
        }
    }
    double get_position(NonnegDouble t) const
    {
        if(t < jerk_duration_)
        {
            return LinearDynamics::get_position(t);
        }
        else
        {
            return LinearDynamics::get_position(jerk_duration_) +
                    LinearDynamics::get_velocity(jerk_duration_) * (t - jerk_duration_) +
                    acceleration_limit_ * 0.5 * pow(t - jerk_duration_, 2);
        }
    }

    template<typename Array>
    Array get_positions(const Array& times) const
    {
        Array positions(times.size());
        for(size_t i = 0; i < times.size(); i++)
        {
            positions[i] = get_position(times[i]);
        }
        return positions;
    }

    Vector find_t_given_velocity(double velocity) const
    {
        Vector potential_solutions =
                LinearDynamics::find_t_given_velocity(velocity);

        Vector solutions(0);
        for(size_t i = 0; i < potential_solutions.size(); i++)
        {
            if(potential_solutions[i] <= jerk_duration_)
            {
                append_to_vector(solutions, potential_solutions[i]);
            }
        }

        double potential_solution = jerk_duration_
                + (velocity - LinearDynamics::get_velocity(jerk_duration_))
                / acceleration_limit_;
        if(potential_solution > jerk_duration_)
        {
            append_to_vector(solutions, potential_solution);
        }


        if(solutions.size() > 2)
        {
            std::cout << "too many solutions, something went wrong!!!"
                      << std::endl;
            exit(-1);
        }
        return solutions;
    }

    bool will_exceed_jointly(const double& max_velocity,
                             const double& max_position) const
    {
        double certificate_time;
        return will_exceed_jointly(max_velocity, max_position, certificate_time);
    }


    bool will_exceed_jointly(const double& max_velocity,
                             const double& max_position,
                             double& certificate_time) const
    {
        if(jerk_ >= 0)
        {
            throw std::domain_error("not implemented for jerk >= 0");
        }
        if(initial_velocity_ < 0)
        {
            throw std::domain_error("not implemented for initial_velocity_ < 0");
        }

        if(max_velocity == std::numeric_limits<double>::infinity() ||
                max_position == std::numeric_limits<double>::infinity())
        {
            return false;
        }

        // find maximum achieved position --------------------------------------
        Vector t_given_zero_velocity =
                find_t_given_velocity(0);
        Vector position_given_zero_velocity =
                get_positions(t_given_zero_velocity);
        if(t_given_zero_velocity.size() != 1)
        {
            std::cout << "something went horribly wrong " << std::endl;
            exit(-1);
        }

        double max_achieved_position = position_given_zero_velocity.maxCoeff();
        if(max_achieved_position < max_position)
        {
            return false;
        }
        if(max_velocity < 0)
        {
            certificate_time = t_given_zero_velocity[0];
            return true;
        }

        Vector t_given_max_velocity =
                find_t_given_velocity(max_velocity);
        Vector position_given_max_velocity =
                get_positions(t_given_max_velocity);

        for(size_t i = 0; i < position_given_max_velocity.size(); i++)
        {
            if(position_given_max_velocity[i] > max_position)
            {
                certificate_time = t_given_max_velocity[i];
                return true;
            }
        }

        return false;
    }








private:
    double acceleration_limit_;
    NonnegDouble jerk_duration_;
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
