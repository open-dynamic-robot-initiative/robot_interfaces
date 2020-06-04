/**
 * @file
 * @brief Observation of a generic n-joint robot.
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
 * @brief Basic observation for a generic n-joint robot.
 *
 * Simple observation type with position, velocity and torque for each joint.
 *
 * @tparam N Number of joints.
 */
template <size_t N>
struct NJointObservation : public Loggable
{
    //! @brief Number of joints.
    static constexpr size_t num_joints = N;

    typedef Eigen::Matrix<double, N, 1> Vector;

    Vector position = Vector::Zero();
    Vector velocity = Vector::Zero();
    Vector torque = Vector::Zero();

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(position, velocity, torque);
    }

    std::vector<std::string> get_name() override
    {
        return {"position", "velocity", "torque"};
    }

    std::vector<std::vector<double>> get_data() override
    {
        typedef std::vector<double> vecd;

        std::vector<vecd> result = {
            vecd(position.size()), vecd(velocity.size()), vecd(torque.size())};

        Vector::Map(&result[0][0], position.size()) = position;
        Vector::Map(&result[1][0], velocity.size()) = velocity;
        Vector::Map(&result[2][0], torque.size()) = torque;

        return result;
    }
};

}  // namespace robot_interfaces
