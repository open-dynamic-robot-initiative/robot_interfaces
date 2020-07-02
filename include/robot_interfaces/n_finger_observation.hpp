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
 * @tparam N_FINGERS  Number of fingers.
 */
template <size_t N_FINGERS>
struct NFingerObservation : public Loggable
{
    static constexpr size_t num_fingers = N_FINGERS;
    static constexpr size_t num_joints = N_FINGERS * 3;

    typedef Eigen::Matrix<double, num_joints, 1> JointVector;
    typedef Eigen::Matrix<double, num_fingers, 1> FingerVector;

    JointVector position = JointVector::Zero();
    JointVector velocity = JointVector::Zero();
    JointVector torque = JointVector::Zero();
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
