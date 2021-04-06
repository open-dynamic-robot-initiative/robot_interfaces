/**
 * @file
 * @brief Helper functions for creating Python bindings.
 * @copyright 2019, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#include <type_traits>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <time_series/pybind11_helper.hpp>

#include <robot_interfaces/robot_frontend.hpp>

namespace robot_interfaces
{
/**
 * @bind Add Python bindings for Types::Observaton::tip_force if it exists.
 *
 * Uses black SFINAE magic to add bindings for "tip_force" if it exists and do
 * nothing if it does not.
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
    static void bind(pybind11::class_<typename Types::Observation> &)
    {
        // tip_force does not exist, so do nothing
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
    }
};

/**
 * \brief Create Python bindings for the specified robot Types.
 *
 * With this function, Python bindings can easily be created for new robots that
 * are based on the NJointRobotTypes.  Example:
 *
 *     PYBIND11_MODULE(py_fortytwo_types, m)
 *     {
 *         create_python_bindings<NJointRobotTypes<42>>(m);
 *     }
 *
 * \tparam Types  An instance of NJointRobotTypes.
 * \param m  The second argument of the PYBIND11_MODULE macro.
 */
template <typename Types>
void create_python_bindings(pybind11::module &m)
{
    pybind11::options options;
    // disable automatic function signature generation as this does not look too
    // nice in the Sphinx documentation.
    options.disable_function_signatures();

    // bindings for the different time series types
    time_series::create_python_bindings<typename Types::Action>(
        m, "_ActionTimeSeries");
    time_series::create_multiprocesses_python_bindings<typename Types::Action>(
        m, "_ActionMultiProcessTimeSeries");
    time_series::create_python_bindings<typename Types::Observation>(
        m, "_ObservationTimeSeries");
    time_series::create_multiprocesses_python_bindings<
        typename Types::Observation>(m, "_ObservationMultiProcessTimeSeries");

    pybind11::class_<typename Types::BaseData, typename Types::BaseDataPtr>(
        m, "BaseData");

    pybind11::class_<typename Types::SingleProcessData,
                     typename Types::SingleProcessDataPtr,
                     typename Types::BaseData>(m, "SingleProcessData")
        .def(pybind11::init<size_t>(), pybind11::arg("history_size") = 1000)
        .def_readonly("desired_action",
                      &Types::SingleProcessData::desired_action)
        .def_readonly("applied_action",
                      &Types::SingleProcessData::applied_action)
        .def_readonly("observation", &Types::SingleProcessData::observation)
        .def_readonly("status", &Types::SingleProcessData::status);

    pybind11::class_<typename Types::MultiProcessData,
                     typename Types::MultiProcessDataPtr,
                     typename Types::BaseData>(m, "MultiProcessData")
        .def(pybind11::init<std::string, bool, size_t>(),
             pybind11::arg("shared_memory_id_prefix"),
             pybind11::arg("is_master"),
             pybind11::arg("history_size") = 1000)
        .def_readonly("desired_action",
                      &Types::MultiProcessData::desired_action)
        .def_readonly("applied_action",
                      &Types::MultiProcessData::applied_action)
        .def_readonly("observation", &Types::MultiProcessData::observation)
        .def_readonly("status", &Types::MultiProcessData::status);

    pybind11::class_<typename Types::Backend, typename Types::BackendPtr>(
        m, "Backend")
        .def("initialize",
             &Types::Backend::initialize,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("request_shutdown",
             &Types::Backend::request_shutdown,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("wait_until_first_action",
             &Types::Backend::wait_until_first_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("wait_until_terminated",
             &Types::Backend::wait_until_terminated,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("is_running",
             &Types::Backend::is_running,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_termination_reason",
             &Types::Backend::get_termination_reason,
             pybind11::call_guard<pybind11::gil_scoped_release>());

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
             pybind11::arg("position_kd") = Types::Action::None());

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

    // Release the GIL when calling any of the front-end functions, so in case
    // there are subthreads running Python, they have a chance to acquire the
    // GIL.
    pybind11::class_<typename Types::Frontend, typename Types::FrontendPtr>(
        m, "Frontend")
        .def(pybind11::init<typename Types::BaseDataPtr>())
        .def("get_observation",
             &Types::Frontend::get_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_desired_action",
             &Types::Frontend::get_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_applied_action",
             &Types::Frontend::get_applied_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_status",
             &Types::Frontend::get_status,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_timestamp_ms",
             &Types::Frontend::get_timestamp_ms,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("append_desired_action",
             &Types::Frontend::append_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("wait_until_timeindex",
             &Types::Frontend::wait_until_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_current_timeindex",
             &Types::Frontend::get_current_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<typename Types::LogEntry>(
        m, "LogEntry", "Represents the logged of one time step.")
        .def_readwrite("timeindex", &Types::LogEntry::timeindex)
        .def_readwrite("timestamp", &Types::LogEntry::timestamp)
        .def_readwrite("status", &Types::LogEntry::status)
        .def_readwrite("observation", &Types::LogEntry::observation)
        .def_readwrite("desired_action", &Types::LogEntry::desired_action)
        .def_readwrite("applied_action", &Types::LogEntry::applied_action);

    pybind11::class_<typename Types::Logger>(m, "Logger")
        .def(pybind11::init<typename Types::BaseDataPtr, int>(),
             pybind11::arg("robot_data"),
             pybind11::arg("block_size") = 100)
        .def("start", &Types::Logger::start)
        .def("stop", &Types::Logger::stop)
        .def("stop_and_save", &Types::Logger::stop)
        .def("reset", &Types::Logger::stop)
        .def("start_continous_writing", &Types::Logger::start)
        .def("stop_continous_writing", &Types::Logger::stop)
        .def("save_current_robot_data",
             &Types::Logger::save_current_robot_data,
             pybind11::arg("filename"),
             pybind11::arg("start_index") = 0,
             pybind11::arg("end_index") = -1)
        .def("save_current_robot_data_binary",
             &Types::Logger::save_current_robot_data_binary,
             pybind11::arg("filename"),
             pybind11::arg("start_index") = 0,
             pybind11::arg("end_index") = -1)
        // raise warining when using deprecated method
        .def("write_current_buffer",
             [](pybind11::object &self,
                const std::string &filename,
                long int start_index,
                long int end_index) {
                 auto warnings = pybind11::module::import("warnings");
                 warnings.attr("warn")(
                     "write_current_buffer() is deprecated, use "
                     "save_current_robot_data() "
                     "instead.");
                 return self.attr("save_current_robot_data")(
                     filename, start_index, end_index);
             },
             pybind11::arg("filename"),
             pybind11::arg("start_index") = 0,
             pybind11::arg("end_index") = -1)
        .def("write_current_buffer_binary",
             [](pybind11::object &self,
                const std::string &filename,
                long int start_index,
                long int end_index) {
                 auto warnings = pybind11::module::import("warnings");
                 warnings.attr("warn")(
                     "write_current_buffer_binary() is deprecated, use "
                     "save_current_robot_data_binary() "
                     "instead.");
                 return self.attr("save_current_robot_data_binary")(
                     filename, start_index, end_index);
             },
             pybind11::arg("filename"),
             pybind11::arg("start_index") = 0,
             pybind11::arg("end_index") = -1);

    pybind11::class_<typename Types::BinaryLogReader,
                     std::shared_ptr<typename Types::BinaryLogReader>>(
        m,
        "BinaryLogReader",
        "BinaryLogReader(filename: str)\n\nSee :meth:`read_file`.")
        .def(pybind11::init<std::string>())
        .def("read_file",
             &Types::BinaryLogReader::read_file,
             pybind11::arg("filename"),
             R"XXX(
                read_file(filename: str)

                Read data from the specified binary robot log file.

                The data is stored to :attr:`data`.

                Args:
                    filename (str): Path to the robot log file.
)XXX")
        .def_readonly("data",
                      &Types::BinaryLogReader::data,
                      "List[LogEntry]: Contains the log entries.");
}

}  // namespace robot_interfaces
