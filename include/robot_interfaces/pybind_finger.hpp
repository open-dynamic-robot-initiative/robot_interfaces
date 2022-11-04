/**
 * @file
 * @brief Functions for creating Python bindings for BLMC CAN robots.
 * @copyright 2019, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#include <type_traits>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <time_series/pybind11_helper.hpp>

#include <robot_interfaces/pybind_helper.hpp>
#include <robot_interfaces/robot_frontend.hpp>
#include <robot_interfaces/types.hpp>

namespace robot_interfaces
{
/**
 * @bind Add Python bindings for Types::Observaton::tip_force if it exists.
 *
 * Uses black SFINAE magic to add bindings for "tip_force" if it exists.
 * Further the pickle functions differ due to this, so handle this here as well.
 *
 * Usage:
 *
 *     BindTipForceIfExists<Types>::bind(pybind_class);
 *
 * This is based on https://stackoverflow.com/a/16000226, see there for an
 * explanation how/why this works.
 */
template <typename Types, typename = int>
struct BindTipForceIfExists
{
    static void bind(pybind11::class_<typename Types::Observation> &c)
    {
        c.def(pybind11::pickle(
            [](const typename Types::Observation &o) {  // __getstate__
                return pybind11::make_tuple(o.position, o.velocity, o.torque);
            },
            [](pybind11::tuple t) {  // __setstate__
                if (t.size() != 3)
                {
                    throw std::runtime_error("Invalid state!");
                }

                typename Types::Observation obs;
                obs.position = t[0].cast<typename Types::Observation::Vector>();
                obs.velocity = t[1].cast<typename Types::Observation::Vector>();
                obs.torque = t[2].cast<typename Types::Observation::Vector>();

                return obs;
            }));
    }
};
template <typename Types>
struct BindTipForceIfExists<Types,
                            decltype((void)Types::Observation::tip_force, 0)>
{
    static void bind(pybind11::class_<typename Types::Observation> &c)
    {
        c.def_readwrite("tip_force",
                        &Types::Observation::tip_force,
                        "Measurements of the push sensors at the finger tips, "
                        "one per finger. Ranges between 0 and 1.");

        c.def(pybind11::pickle(
            [](const typename Types::Observation &o) {  // __getstate__
                return pybind11::make_tuple(
                    o.position, o.velocity, o.torque, o.tip_force);
            },
            [](pybind11::tuple t) {  // __setstate__
                if (t.size() != 4)
                {
                    throw std::runtime_error("Invalid state!");
                }

                typename Types::Observation obs;
                obs.position =
                    t[0].cast<typename Types::Observation::JointVector>();
                obs.velocity =
                    t[1].cast<typename Types::Observation::JointVector>();
                obs.torque =
                    t[2].cast<typename Types::Observation::JointVector>();
                obs.tip_force =
                    t[3].cast<typename Types::Observation::FingerVector>();

                return obs;
            }));
    }
};

/**
 * \brief Create Python bindings for the specified BLMC-CAN-robot Types.
 *
 * With this function, Python bindings can easily be created for new robots that
 * are based on the NJointRobotTypes.  Example:
 *
 *     PYBIND11_MODULE(py_fortytwo_types, m)
 *     {
 *         create_blmc_can_python_bindings<NJointRobotTypes<42>>(m);
 *     }
 *
 * \tparam Types  An instance of NJointRobotTypes.
 * \param m  The second argument of the PYBIND11_MODULE macro.
 */
template <typename Types>
void create_blmc_can_robot_python_bindings(pybind11::module &m)
{
    pybind11::options options;
    // disable automatic function signature generation as this does not look too
    // nice in the Sphinx documentation.
    options.disable_function_signatures();

    create_interface_python_bindings<typename Types::Action,
                                     typename Types::Observation>(m);

    pybind11::class_<typename Types::Action>(m,
                                             "Action",
                                             R"XXX(
                Action(torque=[0] * n_joints, position=[nan] * n_joints, position_kp=[nan] * n_joints, position_kd=[nan] * n_joints)

                Action with desired torque and (optional) position.

                The resulting torque command sent to the robot is::

                    torque_command = torque + PD(position)

                To disable the position controller, set the target position to
                NaN.  The controller is executed joint-wise, so it is possible
                to run it only for some joints by setting a target position for
                these joints and setting the others to NaN.

                The specified torque is always added to the result of the
                position controller, so if you only want to run the position
                controller, make sure to set `torque` to zero for all joints.

                Args:
                    torque:  Desired torque.
                    position:  Desired position.  Set values to NaN to disable
                        position controller for the corresponding joints
                    position_kp:  P-gains for the position controller.  Set to
                        NaN to use default values.
                    position_kd:  D-gains for the position controller.  Set to
                        NaN to use default values.
)XXX")
        .def_readwrite("torque",
                       &Types::Action::torque,
                       "List of desired torques, one per joint.")
        .def_readwrite(
            "position",
            &Types::Action::position,
            "List of desired positions, one per joint.  If set, a PD "
            "position controller is run and the resulting torque is "
            "added to :attr:`torque`.  Set to NaN to disable "
            "position controller (default).")
        .def_readwrite("position_kp",
                       &Types::Action::position_kp,
                       "P-gains for position controller, one per joint.  If "
                       "NaN, default is used.")
        .def_readwrite("position_kd",
                       &Types::Action::position_kd,
                       "D-gains for position controller, one per joint.  If "
                       "NaN, default is used.")
        .def(pybind11::init<typename Types::Action::Vector,
                            typename Types::Action::Vector,
                            typename Types::Action::Vector,
                            typename Types::Action::Vector>(),
             pybind11::arg("torque") = Types::Action::Vector::Zero(),
             pybind11::arg("position") = Types::Action::None(),
             pybind11::arg("position_kp") = Types::Action::None(),
             pybind11::arg("position_kd") = Types::Action::None())
        .def(pybind11::pickle(
            [](const typename Types::Action &a) {  // __getstate__
                // Return a tuple that fully encodes the state of the object
                return pybind11::make_tuple(
                    a.torque, a.position, a.position_kp, a.position_kd);
            },
            [](pybind11::tuple t) {  // __setstate__
                if (t.size() != 4)
                {
                    throw std::runtime_error("Invalid state!");
                }

                // Create a new C++ instance
                typename Types::Action action(
                    t[0].cast<typename Types::Action::Vector>(),
                    t[1].cast<typename Types::Action::Vector>(),
                    t[2].cast<typename Types::Action::Vector>(),
                    t[3].cast<typename Types::Action::Vector>());

                return action;
            }));

    auto obs =
        pybind11::class_<typename Types::Observation>(m, "Observation")
            .def(pybind11::init<>())
            .def_readwrite(
                "position",
                &Types::Observation::position,
                "List of angular joint positions [rad], one per joint.")
            .def_readwrite(
                "velocity",
                &Types::Observation::velocity,
                "List of angular joint velocities [rad/s], one per joint.")
            .def_readwrite("torque",
                           &Types::Observation::torque,
                           "List of torques [Nm], one per joint.");
    BindTipForceIfExists<Types>::bind(obs);
}

}  // namespace robot_interfaces
