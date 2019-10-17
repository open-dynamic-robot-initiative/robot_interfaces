///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cmath>

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

    struct Action
    {
        //! Desired torque command (in addition to position controller).
        Vector torque;
        //! Desired position.  Set to NaN to disable position controller.
        Vector position;
        //! P-gain for position controller.  If NaN, default is used.
        Vector position_kp;
        //! D-gain for position controller.  If NaN, default is used.
        Vector position_kd;


        /**
         * @brief Create action with desired torque and (optional) position.
         *
         * The resulting torque command sent to the robot is
         *
         *     sent_torque = torque + PD(position)
         *
         * To disable the position controller, set the target position to NaN.
         * The controller is executed joint-wise, so it is possible to run it
         * only for some joints by setting a target position for these joints
         * and setting the others to NaN.
         *
         * The specified torque is always added to the result of the position
         * controller, so if you only want to run the position controller, make
         * sure to set `torque` to zero for all joints.
         *
         * For more explicit code, the static factory methods `Troque`,
         * `Position`, `TorqueAndPosition` and `Zero` should be used instead
         * directly creating actions through this constuctor.
         *
         * @param torque  Desired torque.
         * @param position  Desired position.  Set values to NaN to disable
         *     position controller for the corresponding joints
         * @param position_kp  P-gains for the position controller.  Set to NaN
         *     to use default values.
         * @param position_kd  D-gains for the position controller.  Set to NaN
         *     to use default values.
         */
        Action(Vector torque = Vector::Zero(),
               Vector position = None(),
               Vector position_kp = None(),
               Vector position_kd = None())
            : torque(torque),
              position(position),
              position_kp(position_kp),
              position_kd(position_kd)
        {
        }

        /**
         * @brief Create an action that only contains a torque command.
         *
         * @param torque  Desired torque.
         *
         * @return Pure "torque action".
         */
        static Action Torque(Vector torque)
        {
            return Action(torque);
        }

        /**
         * @brief Create an action that only contains a position command.
         *
         * @param position Desired position.
         * @param kp P-gain for position controller.  If not set, default is
         *     used.  Set to NaN for specific joints to use default for this
         *     joint.
         * @param kd D-gain for position controller.  If not set, default is
         *     used.  Set to NaN for specific joints to use default for this
         *     joint.
         *
         * @return Pure "position action".
         */
        static Action Position(Vector position,
                               Vector kp = None(),
                               Vector kd = None())
        {
            return Action(Vector::Zero(), position, kp, kd);
        }

        /**
         * @brief Create an action with both torque and position commands.
         *
         * @param torque Desired torque.
         * @param position Desired position.  Set to NaN for specific joints to
         *     disable position control for this joint.
         * @param kp P-gain for position controller.  If not set, default is
         *     used.  Set to NaN for specific joints to use default for this
         *     joint.
         * @param kd D-gain for position controller.  If not set, default is
         *     used.  Set to NaN for specific joints to use default for this
         *     joint.
         *
         * @return Action with both torque and position commands.
         */
        static Action TorqueAndPosition(Vector torque = Vector::Zero(),
                                        Vector position = None(),
                                        Vector position_kp = None(),
                                        Vector position_kd = None())
        {
            return Action(
                torque, position, position_kp, position_kd);
        }

        /**
         * @brief Create a zero-torque action.
         *
         * @return Zero-torque action with position control disabled.
         */
        static Action Zero()
        {
            return Action();
        }

        /**
         * @brief Create a NaN-Vector.  Helper function to set defaults for
         *     position.
         *
         * @return Vector with all elements set to NaN.
         */
        static Vector None()
        {
            return Vector::Constant(std::numeric_limits<double>::quiet_NaN());
        }
    };

    struct Observation
    {
        Vector position;
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
};  // namespace robot_interfaces

}  // namespace robot_interfaces
