///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include "n_joint_robot_types.hpp"

namespace robot_interfaces
{
/**
 * @brief Collection of functions for a generic N-joint BLMC robot.
 *
 * Defines all the types needed to set up an interface to a generic N-joint BLMC
 * robot that expects as Action a simple vector of N torque commands and
 * provides N observations containing measured joint angle, velocity and torque.
 *
 * Defines some generic functions that can be used by all robots that are based
 * on the `NJointRobotTypes`.
 *
 * @tparam N_JOINTS Number of joints the robot has.
 */
template <size_t N_JOINTS>
struct NJointRobotFunctions
{
    using Types = NJointRobotTypes<N_JOINTS>;

    /**
     * @brief Process the desired action provided by the user.
     *
     * Takes the desired action from the user and does the following processing:
     *
     * ## 1. Run the position controller in case a target position is set.
     *
     *   If the target position is set to a value unequal to NaN for any joint,
     *   a PD position controller is executed for this joint and the resulting
     *   torque command is added to the torque command in the action.
     *
     *   If the P- and/or D-gains are set to a non-NaN value in the action, they
     *   are used for the control.  NaN-values are replaced with the default
     *   gains.
     *
     * ## 2. Apply safety checks.
     *
     *   - Limit the torque to the allowed maximum value.
     *   - Dampen velocity using the given safety_kd gains.  Damping us done
     *     joint-wise using this equation:
     *
     *         torque_damped = torque_desired - safety_kd * current_velocity
     *
     * The resulting action with modifications of all steps is returned.
     *
     * @param desired_action  Desired action given by the user.
     * @param latest_observation  Latest observation from the robot.
     * @param max_torque_Nm  Maximum allowed absolute torque.
     * @param safety_kd  D-gain for velocity damping.
     * @param default_position_control_kp  Default P-gain for position control.
     * @param default_position_control_kd  Default D-gain for position control.
     *
     * @return Resulting action after applying all the processing.
     */
    static typename Types::Action process_desired_action(
        const typename Types::Action &desired_action,
        const typename Types::Observation &latest_observation,
        const double max_torque_Nm,
        const typename Types::Action::Vector &safety_kd,
        const typename Types::Action::Vector &default_position_control_kp,
        const typename Types::Action::Vector &default_position_control_kd)
    {
        typename Types::Action processed_action;

        processed_action.torque = desired_action.torque;
        processed_action.position = desired_action.position;

        // Position controller
        // -------------------
        // TODO: add position limits

        // Run the position controller only if a target position is set for at
        // least one joint.
        if (!processed_action.position.array().isNaN().all())
        {
            // Replace NaN-values with default gains
            processed_action.position_kp =
                desired_action.position_kp.array().isNaN().select(
                    default_position_control_kp, desired_action.position_kp);
            processed_action.position_kd =
                desired_action.position_kd.array().isNaN().select(
                    default_position_control_kd, desired_action.position_kd);

            typename Types::Action::Vector position_error =
                processed_action.position - latest_observation.position;

            // simple PD controller
            typename Types::Action::Vector position_control_torque =
                processed_action.position_kp.cwiseProduct(position_error) -
                processed_action.position_kd.cwiseProduct(
                    latest_observation.velocity);

            // position_control_torque contains NaN for joints where target
            // position is set to NaN!  Filter those out and set the torque to
            // zero instead.
            position_control_torque =
                position_control_torque.array().isNaN().select(
                    0, position_control_torque);

            // Add result of position controller to the torque command
            processed_action.torque += position_control_torque;
        }

        // Safety Checks
        // -------------
        // limit to configured maximum torque
        processed_action.torque =
            mct::clamp(processed_action.torque, -max_torque_Nm, max_torque_Nm);
        // velocity damping to prevent too fast movements
        processed_action.torque -=
            safety_kd.cwiseProduct(latest_observation.velocity);
        // after applying checks, make sure we are still below the max. torque
        processed_action.torque =
            mct::clamp(processed_action.torque, -max_torque_Nm, max_torque_Nm);

        return processed_action;
    }
};

}  // namespace robot_interfaces
