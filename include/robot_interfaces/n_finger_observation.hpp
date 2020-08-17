/**
 * @file
 * @brief Observation of a Finger robot.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <serialization_utils/cereal_eigen.hpp>

#include <robot_interfaces/loggable.hpp>

namespace robot_interfaces
{
/**
 * @brief Observation of a Finger robot.
 *
 *
 * Values like angular position, velocity or torque of the joints are
 * represented as a 1-dimensional vector with one element per joint.  The order
 * of the joints in these vectors is as follows:
 *
 *     0. Finger 1, upper joint
 *     1. Finger 1, middle joint
 *     2. Finger 1, lower joint
 *     3. Finger 2, upper joint
 *     4. Finger 2, middle joint
 *     5. Finger 2, lower joint
 *     ...
 *     #. Finger n, upper joint
 *     #. Finger n, middle joint
 *     #. Finger n, lower joint
 *
 * @tparam N_FINGERS  Number of fingers.
 */
template <size_t N_FINGERS>
struct NFingerObservation : public Loggable
{
    static constexpr size_t num_fingers = N_FINGERS;
    static constexpr size_t num_joints = N_FINGERS * 3;

    typedef Eigen::Matrix<double, num_joints, 1> JointVector;
    typedef Eigen::Matrix<double, num_fingers, 1> FingerVector;

    //! @brief Measured angular position of all joints in radian.
    JointVector position = JointVector::Zero();

    //! @brief Measured velocity of all joints in radian/second.
    JointVector velocity = JointVector::Zero();

    //! @brief Measured torques of all joints in Nm.
    JointVector torque = JointVector::Zero();

    /**
     * @brief Measurements of the pressure sensors at the finger tips.
     *
     * One per finger.  Ranges between 0 and 1 without specific unit.  Note that
     * not all fingers are equipped with an actual sensor!  For fingers without
     * sensor, this value is undefined.
     */
    FingerVector tip_force = FingerVector::Zero();

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(position, velocity, torque, tip_force);
    }

    std::vector<std::string> get_name() override
    {
        return {"position", "velocity", "torque", "tip_force"};
    }

    std::vector<std::vector<double>> get_data() override
    {
        typedef std::vector<double> vecd;

        std::vector<vecd> result = {vecd(position.size()),
                                    vecd(velocity.size()),
                                    vecd(torque.size()),
                                    vecd(tip_force.size())};

        JointVector::Map(&result[0][0], position.size()) = position;
        JointVector::Map(&result[1][0], velocity.size()) = velocity;
        JointVector::Map(&result[2][0], torque.size()) = torque;
        FingerVector::Map(&result[3][0], tip_force.size()) = tip_force;

        return result;
    }
};
}  // namespace robot_interfaces
