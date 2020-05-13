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
constexpr size_t JOINTS_PER_FINGER = 3;
constexpr size_t BOARDS_PER_FINGER = 2;

/**
 * @brief Types for the Finger robot (basic 3-joint robot).
 */
template <size_t N_FINGERS>
struct FingerTypes
    : public RobotInterfaceTypes<NJointAction<N_FINGERS * JOINTS_PER_FINGER>,
                                 FingerObservation<N_FINGERS>>
{
};


// typedefs for common number of fingers
typedef FingerTypes<1> MonoFingerTypes;
typedef FingerTypes<3> TriFingerTypes;

}  // namespace robot_interfaces
