///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>

namespace robot_interfaces
{
/**
 * @brief Driver for interfacing the actual robot hardware or simulation.
 *
 * Interface to the robot used by the subsequent classes. Any robot (be it real
 * or simulation) has to derive from this class and implement the functions
 * apply_action(), get_latest_observation() and shutdown().
 * This Base class provides some timing logic around those three functions. It
 * makes sure that after the first call of apply_action(), it is always called
 * again after some specified time, otherwise the shutdown() method will
 * be called. This Base class also makes sure that the apply_action() function
 * itself does not take more time than expected.
 *
 * @tparam Action
 * @tparam Observation
 */
template <typename Action, typename Observation>
class RobotDriver
{
public:
    /**
     * @brief Initialize the robot.
     *
     * Any initialization procedures that need to be done before sending
     * actions to the robot should be done in this method (e.g. homing to find
     * the absolute position).
     */
    virtual void initialize() = 0;

    /**
     * @brief Apply action immediately and block until it is executed.
     *
     * This method must apply the desired_action immediately when it is called,
     * and only return once the action has been executed completely.  This way
     * we can accommodate both simulators and real robots with this interface.
     *
     * @param desired_action  The action we want to apply.
     * @return  The action that was actually applied (since due to safety
     *     reasons it might not be possible to apply the desired action).
     */
    virtual Action apply_action(const Action &desired_action) = 0;

    /**
     * @brief Return the latest observation immediately.
     *
     * @return Observation
     */
    virtual Observation get_latest_observation() = 0;

    /**
     * @brief Get error message if there is any error.
     *
     * @return Returns an error message or an empty string if there is no error.
     */
    virtual std::string get_error() = 0;

    /**
     * @brief Shut down the robot safely.
     */
    virtual void shutdown() = 0;
};

}  // namespace robot_interfaces
