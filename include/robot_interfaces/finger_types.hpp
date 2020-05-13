///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <robot_interfaces/n_joint_robot_types.hpp>

namespace robot_interfaces
{
/**
 * @brief Types for the Finger robot (basic 3-joint robot).
 */
struct FingerTypes : public SimpleNJointRobotTypes<3>
{
};

}  // namespace robot_interfaces
