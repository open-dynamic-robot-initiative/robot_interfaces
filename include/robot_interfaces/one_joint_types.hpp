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
 * @brief Types for the One-Joint robot.
 */
struct OneJointTypes : public NJointRobotTypes<1>
{
};

}  // namespace robot_interfaces


namespace YAML
{
template <>
struct convert<robot_interfaces::OneJointTypes::Vector>
    : robot_interfaces::OneJointTypes::yaml_convert_vector
{
};
}  // namespace YAML

