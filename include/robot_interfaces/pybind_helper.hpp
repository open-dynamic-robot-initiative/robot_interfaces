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
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>


namespace robot_interfaces
{

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
    pybind11::class_<typename Types::Data,
        typename Types::DataPtr>(m, "Data")
            .def(pybind11::init<>());

    pybind11::class_<typename Types::Backend,
        typename Types::BackendPtr>(m, "Backend")
            .def("initialize", &Types::Backend::initialize);

    pybind11::class_<typename Types::Action>(m, "Action")
        .def_readwrite("torque", &Types::Action::torque)
        .def_readwrite("position", &Types::Action::position)
        .def_readwrite("position_kp", &Types::Action::position_kp)
        .def_readwrite("position_kd", &Types::Action::position_kd)
        .def(pybind11::init<typename Types::Vector,
                            typename Types::Vector,
                            typename Types::Vector,
                            typename Types::Vector>(),
             pybind11::arg("torque") = Types::Vector::Zero(),
             pybind11::arg("position") = Types::Action::None(),
             pybind11::arg("position_kp") = Types::Action::None(),
             pybind11::arg("position_kd") = Types::Action::None());

    pybind11::class_<typename Types::Observation>(m, "Observation")
        .def(pybind11::init<>())
        .def_readwrite("position", &Types::Observation::position)
        .def_readwrite("velocity", &Types::Observation::velocity)
        .def_readwrite("torque", &Types::Observation::torque);

    pybind11::class_<typename Types::Frontend, typename Types::FrontendPtr>(m, "Frontend")
        .def(pybind11::init<typename Types::DataPtr>())
        .def("get_observation", &Types::Frontend::get_observation)
        .def("get_desired_action", &Types::Frontend::get_desired_action)
        .def("get_applied_action", &Types::Frontend::get_applied_action)
        .def("get_status", &Types::Frontend::get_status)
        .def("get_time_stamp_ms", &Types::Frontend::get_time_stamp_ms)
        .def("append_desired_action", &Types::Frontend::append_desired_action)
        .def("wait_until_time_index", &Types::Frontend::wait_until_timeindex)
        .def("get_current_time_index", &Types::Frontend::get_current_timeindex);

    pybind11::class_<typename Types::Logger>(m, "Logger")
        .def(pybind11::init<typename Types::DataPtr, int>())
        .def("start", &Types::Logger::start)
        .def("stop", &Types::Logger::stop);
}

}  // namespace
