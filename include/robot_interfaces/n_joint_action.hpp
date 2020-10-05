/**
 * @file
 * @brief Action of a generic n-joint robot.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <serialization_utils/cereal_eigen.hpp>

#include <robot_interfaces/loggable.hpp>

namespace robot_interfaces
{
/**
 * @brief Action of a generic n-joint robot.
 *
 * This action type can be used for all n-joint robots that expect torque or
 * position commands on joint-level.
 *
 * @tparam N Number of joints.
 */
template <size_t N>
struct NJointAction : public Loggable
{
    //! @brief Number of joints.
    static constexpr size_t num_joints = N;

    typedef Eigen::Matrix<double, N, 1> Vector;

    //! Desired torque command (in addition to position controller).
    Vector torque;
    //! Desired position.  Set to NaN to disable position controller.
    Vector position;
    //! P-gain for position controller.  If NaN, default is used.
    Vector position_kp;
    //! D-gain for position controller.  If NaN, default is used.
    Vector position_kd;

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(torque, position, position_kp, position_kd);
    }

    std::vector<std::string> get_name() override
    {
        return {"torque", "position", "position_kp", "position_kd"};
    }

    std::vector<std::vector<double>> get_data() override
    {
        // first map the Eigen vectors to std::vectors
        std::vector<double> torque_;
        torque_.resize(torque.size());
        Vector::Map(&torque_[0], torque.size()) = torque;

        std::vector<double> position_;
        position_.resize(position.size());
        Vector::Map(&position_[0], position.size()) = position;

        std::vector<double> position_kp_;
        position_kp_.resize(position_kp.size());
        Vector::Map(&position_kp_[0], position_kp.size()) = position_kp;

        std::vector<double> position_kd_;
        position_kd_.resize(position_kd.size());
        Vector::Map(&position_kd_[0], position_kd.size()) = position_kd;

        // then return them in a fixed size vector of vectors to avoid
        // copying due to pushing back value of information!
        std::vector<std::vector<double>> result;
        result = {torque_, position_, position_kp_, position_kd_};

        return result;
    }

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
     * For more explicit code, the static factory methods `Torque`,
     * `Position`, `TorqueAndPosition` and `Zero` should be used instead
     * directly creating actions through this constructor.
     *
     * @param torque  Desired torque.
     * @param position  Desired position.  Set values to NaN to disable
     *     position controller for the corresponding joints
     * @param position_kp  P-gains for the position controller.  Set to NaN
     *     to use default values.
     * @param position_kd  D-gains for the position controller.  Set to NaN
     *     to use default values.
     */
    NJointAction(Vector torque = Vector::Zero(),
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
    static NJointAction Torque(Vector torque)
    {
        return NJointAction(torque);
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
    static NJointAction Position(Vector position,
                                 Vector kp = None(),
                                 Vector kd = None())
    {
        return NJointAction(Vector::Zero(), position, kp, kd);
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
    static NJointAction TorqueAndPosition(Vector torque,
                                          Vector position,
                                          Vector position_kp = None(),
                                          Vector position_kd = None())
    {
        return NJointAction(torque, position, position_kp, position_kd);
    }

    /**
     * @brief Create a zero-torque action.
     *
     * @return Zero-torque action with position control disabled.
     */
    static NJointAction Zero()
    {
        return NJointAction();
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

}  // namespace robot_interfaces
