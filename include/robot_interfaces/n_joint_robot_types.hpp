///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Eigen>

#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_frontend.hpp>

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
struct NJointRobotTypes
{
    typedef Eigen::Matrix<double, N, 1> Vector;

    typedef Vector Action;
    struct Observation
    {
        Vector angle;
        Vector velocity;
        Vector torque;
    };

    typedef RobotBackend<Action, Observation> Backend;
    typedef std::shared_ptr<Backend> BackendPtr;
    typedef typename Backend::Status Status;

    typedef RobotData<Action, Observation, Status> Data;
    typedef std::shared_ptr<Data> DataPtr;

    typedef RobotFrontend<Action, Observation> Frontend;
    typedef std::shared_ptr<Frontend> FrontendPtr;
};

}  // namespace robot_interfaces
