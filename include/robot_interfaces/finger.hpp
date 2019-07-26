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





namespace robot_interfaces
{





class NewFinger
{
public:
    enum JointIndexing {base, center, tip, joint_count};

    typedef Eigen::Vector3d Vector;

    typedef Eigen::Vector3d Action;


//     struct Observation
//     {
//         Vector angle;
//         Vector velocity;
//         Vector torque;
//     };
//     struct Data
//     {
//         Action desired_action;
//         Action safe_action;
//         Observation observation;
//     };


//     NewFinger() 
//     { }

//     /**
//      * @brief this function will
//      * wait until the previous step is completed,
//      * store the latest observation in data,
//      * compute the safe_action based on the desired_action,
//      * send the safe_action to the robot,
//      * store both actions in data,
//      * return data
//      */
//     virtual Data step(Action desired_action)
//     {
//         desired_action_->append(desired_action);
//         long int t = desired_action_->newest_timeindex();

//         Data data;
//         data.desired_action = (*desired_action_)[t];
//         data.safe_action =(*safe_action_)[t];
//         data.observation = (*observation_)[t];

//         return data;
//     }

// protected:
//     // timing ------------------------------------------------------------------
//     virtual bool is_realtime()
//     {
//         return true; /// TODO: this has to be implemented in the child class
//     }
//     virtual double step_duration_ms()
//     {
//         return 1.0; /// TODO: this has to be implemented in the child class
//     }

//     // getting observations and setting controls -------------------------------
//     virtual Observation get_latest_observation()
//     {
//         return Observation();
//     }

//     virtual void apply_action(Action action)
//     {
//         apply_torques(action);
//         real_time_tools::Timer::sleep_ms(1.0); /// TODO this is temporary
//     }


//     virtual Action constrain_action(Action desired_action)
//     {
//         return constrain_torques(desired_action); ///TODO: this is temporary
//     }

// private:
//     std::shared_ptr<real_time_tools::ThreadsafeTimeseries<Vector>> desired_action_;
//     std::shared_ptr<real_time_tools::ThreadsafeTimeseries<Action>> safe_action_;
//     std::shared_ptr<real_time_tools::ThreadsafeTimeseries<Observation>> observation_;

//     void loop()
//     {
//         for(long int t = 0; true; t++)
//         {
//             safe_action_->append(constrain_action((*desired_action_)[t]));
//             observation_->append(get_latest_observation());
//             apply_action((*safe_action_)[t]);

//             check_timing(observation_->timestamp_ms(t) -
//                          observation_->timestamp_ms(t-1));
//         }
//     }

//     void check_timing(double delta_time_ms)
//     {
//         if(is_realtime() &&
//                 (delta_time_ms > step_duration_ms() * 1.1 ||
//                  delta_time_ms < step_duration_ms() * 0.9))
//         {
//             std::cout << "control loop did not run at expected rate "
//                          "did you provide actions fast enough?"
//                       << std::endl;
//             exit(-1);
//         }
//     }




};







class Finger: public NewFinger
{
public:
    //typedef Eigen::Vector3d Vector;
    typedef Eigen::Vector4d Quaternion; // should go away

    //enum JointIndexing {base, center, tip, joint_count};

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
