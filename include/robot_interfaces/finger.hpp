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
        if(!std::isnan(value) && value < 0.0)
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


double clamp(const double& value, const double& limit_a, const double& limit_b)
{
    if(limit_b > limit_a)
        return std::max(limit_a, std::min(value, limit_b));

    return std::max(limit_b, std::min(value, limit_a));

}




template<typename Vector>
void append_to_vector(Vector& vector,
                      const double& element)
{
    vector.conservativeResize(vector.size() + 1);
    vector[vector.size() - 1] = element;
}


template<typename Matrix>
void append_rows_to_matrix(Matrix& matrix,
                           const Matrix& rows)
{
    if(matrix.cols() != rows.cols())
        throw std::invalid_argument("need to have same number of cols");

    matrix.conservativeResize(matrix.rows() + rows.rows(), matrix.cols());
    matrix.bottomRows(rows.rows()) = rows;
}


class LinearDynamics
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 10> Vector;

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


bool approx_equal(double x, double y, double epsilon = 1e-10)
{
    return (std::fabs(x - y) < epsilon);
}

template<typename Vector>
bool contains(Vector v, double x)
{
    for(size_t i = 0; i < v.size(); i++)
    {
        if(approx_equal(v[i], x))
        {
            return true;
        }
    }
    return false;
}



class LinearDynamicsWithAccelerationConstraint: public LinearDynamics
{
public:

    void print_parameters() const
    {
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "jerk: " << jerk_ << std::endl
                  << "initial_acceleration: " << initial_acceleration_ << std::endl
                  << "initial_velocity: " << initial_velocity_ << std::endl
                  << "initial_position: " << initial_position_ << std::endl
                  << "acceleration_limit: " << acceleration_limit_ << std::endl
                  << "jerk_duration: " << jerk_duration_ << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
    }

    typedef LinearDynamics::Vector Vector;

    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 10, 10>
    Matrix;


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

        set_initial_acceleration(initial_acceleration);
    }


    void set_initial_acceleration(double initial_acceleration)
    {
        if(std::fabs(initial_acceleration) > std::fabs(acceleration_limit_))
            throw std::invalid_argument("expected "
                                        "std::fabs(initial_acceleration) > "
                                        "abs_acceleration_limit");
        initial_acceleration_ = initial_acceleration;
        jerk_duration_ =
                (acceleration_limit_ - initial_acceleration_) / jerk_;
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
        if(potential_solution > jerk_duration_ &&
                !(contains(solutions, jerk_duration_) &&
                  approx_equal(potential_solution, jerk_duration_)))
        {
            append_to_vector(solutions, potential_solution);
        }



        if(solutions.size() > 2)
        {
            std::cout << "too many solutions, something went wrong!!!"
                      << std::endl;
            print_parameters();

            std::cout << "potential_solutions[0]: " << potential_solutions[0] << std::endl;

            std::cout << "potential_solutions size: " << potential_solutions.size() << " content: " << potential_solutions.transpose() << std::endl;


            std::cout << "solutions size: " << solutions.size() << " content: " << solutions.transpose() << std::endl;
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
        if(max_velocity == std::numeric_limits<double>::infinity() ||
                max_position == std::numeric_limits<double>::infinity())
        {
            return false;
        }
        if(jerk_ > 0)
        {
            certificate_time = std::numeric_limits<double>::infinity();
            return true;
        }
        if(jerk_ == 0)
        {
            throw std::domain_error("not implemented for jerk == 0");
        }

        // find maximum achieved position --------------------------------------
        ///\todo we could do this in a cleaner way with candidate points
        //        Matrix candidate_points(0, 0);
        //        Vector candidate_times(0);

        //        append_rows_to_matrix(candidate_points,
        //                              Eigen::Vector2d(initial_velocity_,
        //                                              initial_position_).transpose());
        //        append_to_vector(candidate_times, 0);


        if(initial_velocity_ > max_velocity &&
                initial_position_ > max_position)
        {
            certificate_time = 0;
            return true;
        }

        Vector t_given_zero_velocity = find_t_given_velocity(0);
        if(t_given_zero_velocity.size() > 0)
        {
            Vector position_given_zero_velocity =
                    get_positions(t_given_zero_velocity);

            Vector::Index max_index;
            double max_achieved_position =
                    position_given_zero_velocity.maxCoeff(&max_index);
            if(max_achieved_position < max_position)
            {
                return false;
            }
            if(max_velocity < 0)
            {
                certificate_time = t_given_zero_velocity[max_index];
                return true;
            }
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


    bool will_deceed_jointly(const double& min_velocity,
                             const double& min_position) const
    {
        double certificate_time;
        return will_deceed_jointly(min_velocity, min_position, certificate_time);
    }

    bool will_deceed_jointly(const double& min_velocity,
                             const double& min_position,
                             double& certificate_time) const
    {
        LinearDynamicsWithAccelerationConstraint
                flipped_dynamics(-jerk_,
                                 -initial_acceleration_,
                                 -initial_velocity_,
                                 -initial_position_,
                                 std::fabs(acceleration_limit_));

        return flipped_dynamics.will_exceed_jointly(-min_velocity,
                                                    -min_position,
                                                    certificate_time);
    }


private:
    double acceleration_limit_;
    NonnegDouble jerk_duration_;
};



double find_max_admissible_acceleration(
        const double& initial_velocity,
        const double& initial_position,
        const double& max_velocity,
        const double& max_position,
        const NonnegDouble& abs_jerk_limit,
        const NonnegDouble& abs_acceleration_limit)
{
    double lower = -abs_acceleration_limit;
    double upper = abs_acceleration_limit;


    LinearDynamicsWithAccelerationConstraint dynamics(-abs_jerk_limit,
                                                      lower,
                                                      initial_velocity,
                                                      initial_position,
                                                      abs_acceleration_limit);


    if(dynamics.will_exceed_jointly(max_velocity, max_position))
    {
        /// \todo: not quite sure what is the right thing to do here
        return lower;
    }

    dynamics.set_initial_acceleration(upper);
    if(!dynamics.will_exceed_jointly(max_velocity, max_position))
    {
        return upper;
    }

    for(size_t i = 0; i < 20; i++)
    {
        double middle = (lower + upper) / 2.0;

        dynamics.set_initial_acceleration(middle);
        if(dynamics.will_exceed_jointly(max_velocity, max_position))
        {
            upper = middle;
        }
        else
        {
            lower = middle;
        }
    }
    return lower;
}



double find_min_admissible_acceleration(
        const double& initial_velocity,
        const double& initial_position,
        const double& min_velocity,
        const double& min_position,
        const NonnegDouble& abs_jerk_limit,
        const NonnegDouble& abs_acceleration_limit)
{
    return -find_max_admissible_acceleration(-initial_velocity,
                                             -initial_position,
                                             -min_velocity,
                                             -min_position,
                                             abs_jerk_limit,
                                             abs_acceleration_limit);
}




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


class SafetyConstraint
{
public:
    SafetyConstraint()
    {
        min_velocity_ = -std::numeric_limits<double>::infinity();
        min_position_ = -std::numeric_limits<double>::infinity();
        max_velocity_ = std::numeric_limits<double>::infinity();
        max_position_ = std::numeric_limits<double>::infinity();
        max_torque_ = 1.0;
        max_jerk_ = 1.0;
        inertia_ = 1.0;
    }

    SafetyConstraint(double min_velocity,
                     double min_position,
                     double max_velocity,
                     double max_position,
                     NonnegDouble max_torque,
                     NonnegDouble max_jerk,
                     NonnegDouble inertia)
    {
        min_velocity_ = min_velocity;
        min_position_ = min_position;
        max_velocity_ = max_velocity;
        max_position_ = max_position;
        max_torque_ = max_torque;
        max_jerk_ = max_jerk;
        inertia_ = inertia;
    }


    double get_safe_torque(const double& torque,
                           const double& velocity,
                           const double& position)
    {
        double safe_torque = clamp(torque, -max_torque_, max_torque_);

//        std::cout << "safe_torque: " << safe_torque << std::endl;


        NonnegDouble max_achievable_acc = max_torque_ / inertia_;




        double max_admissible_acc =
                find_max_admissible_acceleration(velocity,
                                                 position,
                                                 max_velocity_,
                                                 max_position_,
                                                 max_jerk_,
                                                 max_achievable_acc);
        double max_admissible_torque = max_admissible_acc * inertia_;



//        LinearDynamicsWithAccelerationConstraint
//                test_dynamics(- max_jerk_,
//                              max_achievable_acc,
//                              velocity,
//                              position,
//                              max_achievable_acc);

//        test_dynamics.print_parameters();
//        std::cout << "will exceed: " << test_dynamics.will_exceed_jointly(max_velocity_, max_position_) << std::endl;
//        std::cout << "max_admissible_acc: " << max_admissible_acc << std::endl;


//        std::cout << "max_achievable_acc: " << max_achievable_acc << std::endl;
//        std::cout << "max_admissible_acc: " << max_admissible_acc << std::endl;
//        std::cout << "max_admissible_torque: " << max_admissible_torque << std::endl;



        double min_admissible_acc =
                find_min_admissible_acceleration(velocity,
                                                 position,
                                                 min_velocity_,
                                                 min_position_,
                                                 max_jerk_,
                                                 max_achievable_acc);
        double min_admissible_torque = min_admissible_acc * inertia_;

//        std::cout << "min_admissible_acc: " << min_admissible_acc << std::endl;
//        std::cout << "min_admissible_torque: " << min_admissible_torque << std::endl;



        if(min_admissible_torque > max_admissible_torque)
        {
            std::cout << "min_admissible_torque > max_admissible_torque!!!!"
                      << std::endl;
            return 0;
        }

        safe_torque = clamp(safe_torque,
                            min_admissible_torque, max_admissible_torque);


        if(safe_torque > max_torque_ || safe_torque < -max_torque_)
        {
            std::cout << "something went horribly horribly wrong " << std::endl;
            return 0;
        }

        return safe_torque;
    }

    double min_velocity_;
    double min_position_;
    double max_velocity_;
    double max_position_;
    NonnegDouble max_torque_;
    NonnegDouble max_jerk_;
    NonnegDouble inertia_;
};



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

    virtual void wait_for_execution() const = 0; /// \todo: this can be implemented here

    Vector get_max_torques() const
    {
        return max_torque_ * Vector::Ones();
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
    std::array<SafetyConstraint, 3> safety_constraints_;
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

        safety_constraints_[table].min_velocity_ = -3.0;
        safety_constraints_[table].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[table].max_velocity_ = 3.0;
        safety_constraints_[table].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[table].max_torque_ = max_torques_[table];
        safety_constraints_[table].inertia_ = 1.0; //dummy
        safety_constraints_[base].max_jerk_ = 10000.0; // dummy

        safety_constraints_[base].min_velocity_ = -10.0;
        safety_constraints_[base].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_velocity_ = 10.0;
        safety_constraints_[base].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_torque_ = max_torques_[base];
        safety_constraints_[base].inertia_ = 1.0; //dummy
        safety_constraints_[base].max_jerk_ =  10000.0; // dummy


        safety_constraints_[tip].min_velocity_ = -10.0;
        safety_constraints_[tip].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_velocity_ = 10.0;
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
    std::array<SafetyConstraint, 3> safety_constraints_;
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
