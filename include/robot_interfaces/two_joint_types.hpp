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
 * @brief Types for the Two-Joint robot.
 */
struct TwoJointTypes : public NJointRobotTypes<2>
{
};

}  // namespace robot_interfaces


namespace YAML
{
template <>
struct convert<robot_interfaces::TwoJointTypes::Vector>
    : robot_interfaces::TwoJointTypes::yaml_convert_vector
{
};
}  // namespace YAML


