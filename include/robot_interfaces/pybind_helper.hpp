/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file
 * \brief Helper functions for creating Python bindings.
 */
#include <type_traits>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

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
        c.def_readwrite("tip_force", &Types::Observation::tip_force);
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
    pybind11::class_<typename Types::BaseData, typename Types::BaseDataPtr>(
        m, "BaseData");

    pybind11::class_<typename Types::SingleProcessData,
                     typename Types::SingleProcessDataPtr,
                     typename Types::BaseData>(m, "SingleProcessData")
        .def(pybind11::init<size_t>(), pybind11::arg("history_size") = 1000);

    pybind11::class_<typename Types::MultiProcessData,
                     typename Types::MultiProcessDataPtr,
                     typename Types::BaseData>(m, "MultiProcessData")
        .def(pybind11::init<std::string, bool, size_t>(),
             pybind11::arg("shared_memory_id_prefix"),
             pybind11::arg("is_master"),
             pybind11::arg("history_size") = 1000);

    pybind11::class_<typename Types::Backend, typename Types::BackendPtr>(
        m, "Backend")
        .def("initialize",
             &Types::Backend::initialize,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("wait_until_first_action",
             &Types::Backend::wait_until_first_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("wait_until_terminated",
             &Types::Backend::wait_until_terminated,
             pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<typename Types::Action>(m, "Action")
        .def_readwrite("torque", &Types::Action::torque)
        .def_readwrite("position", &Types::Action::position)
        .def_readwrite("position_kp", &Types::Action::position_kp)
        .def_readwrite("position_kd", &Types::Action::position_kd)
        .def(pybind11::init<typename Types::Action::Vector,
                            typename Types::Action::Vector,
                            typename Types::Action::Vector,
                            typename Types::Action::Vector>(),
             pybind11::arg("torque") = Types::Action::Vector::Zero(),
             pybind11::arg("position") = Types::Action::None(),
             pybind11::arg("position_kp") = Types::Action::None(),
             pybind11::arg("position_kd") = Types::Action::None());

    auto obs = pybind11::class_<typename Types::Observation>(m, "Observation")
                   .def(pybind11::init<>())
                   .def_readwrite("position", &Types::Observation::position)
                   .def_readwrite("velocity", &Types::Observation::velocity)
                   .def_readwrite("torque", &Types::Observation::torque);
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
        // TODO get_time_stamp_ms is deprecated, remove it some time in the near
        // future.
        .def(
            "get_time_stamp_ms",
            [](pybind11::object &self, const TimeIndex &t) {
                auto warnings = pybind11::module::import("warnings");
                warnings.attr("warn")(
                    "get_time_stamp_ms() is deprecated, use get_timestamp_ms() "
                    "instead.");
                return self.attr("get_timestamp_ms")(t);
            })
        .def("get_timestamp_ms",
             &Types::Frontend::get_timestamp_ms,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("append_desired_action",
             &Types::Frontend::append_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        // TODO: deprecated, remove in near future
        .def("wait_until_time_index",
             [](pybind11::object &self, const TimeIndex &t) {
                 auto warnings = pybind11::module::import("warnings");
                 warnings.attr("warn")(
                     "wait_until_time_index() is deprecated, use "
                     "wait_until_timeindex() instead.");
                 return self.attr("wait_until_timeindex")(t);
             })
        .def("wait_until_timeindex",
             &Types::Frontend::wait_until_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        // TODO: deprecated, remove in near future
        .def("get_current_time_index",
             [](pybind11::object &self) {
                 auto warnings = pybind11::module::import("warnings");
                 warnings.attr("warn")(
                     "get_current_time_index() is deprecated, use "
                     "get_current_timeindex() instead.");
                 return self.attr("get_current_timeindex")();
             })
        .def("get_current_timeindex",
             &Types::Frontend::get_current_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<typename Types::LogEntry>(m, "LogEntry")
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
        .def("write_current_buffer",
             &Types::Logger::write_current_buffer,
             pybind11::arg("filename"),
             pybind11::arg("start_index") = 0,
             pybind11::arg("end_index") = -1)
        .def("write_current_buffer_binary",
             &Types::Logger::write_current_buffer_binary,
             pybind11::arg("filename"),
             pybind11::arg("start_index") = 0,
             pybind11::arg("end_index") = -1);

    pybind11::class_<typename Types::BinaryLogReader,
                     std::shared_ptr<typename Types::BinaryLogReader>>(
        m, "BinaryLogReader")
        .def(pybind11::init<std::string>())
        .def("read_file", &Types::BinaryLogReader::read_file)
        .def_readonly("data", &Types::BinaryLogReader::data);
}

}  // namespace robot_interfaces
