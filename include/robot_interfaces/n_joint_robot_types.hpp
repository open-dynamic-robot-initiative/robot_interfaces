/**
 * @file
 * @brief Types for an n-joint robot.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include "n_finger_observation.hpp"
#include "n_joint_action.hpp"
#include "n_joint_observation.hpp"
#include "types.hpp"

namespace robot_interfaces
{
/**
 * @brief Collection of types for a generic N-joint BLMC robot.
 *
 * Defines all the types needed to set up an interface to a generic N-joint BLMC
 * robot that expects as Action a simple vector of N torque commands and
 * provides N observations containing measured joint angle, velocity and torque.
 *
 * @tparam N Number of joints
 */
template <size_t N>
struct SimpleNJointRobotTypes
    : public RobotInterfaceTypes<NJointAction<N>, NJointObservation<N>>
{
};

}  // namespace robot_interfaces
