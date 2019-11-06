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

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <robot_interfaces/two_joint_types.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_two_joint_types, m)
{
    pybind11::class_<robot_interfaces::TwoJointTypes::Data,
        robot_interfaces::TwoJointTypes::DataPtr>(m, "Data")
            .def(pybind11::init<>());

    pybind11::class_<robot_interfaces::TwoJointTypes::Backend,
        robot_interfaces::TwoJointTypes::BackendPtr>(m, "Backend")
            .def("initialize", &robot_interfaces::TwoJointTypes::Backend::initialize);

    pybind11::class_<TwoJointTypes::Action>(m, "Action")
        .def_readwrite("torque", &TwoJointTypes::Action::torque)
        .def_readwrite("position", &TwoJointTypes::Action::position)
        .def_readwrite("position_kp", &TwoJointTypes::Action::position_kp)
        .def_readwrite("position_kd", &TwoJointTypes::Action::position_kd)
        .def(
            pybind11::init<TwoJointTypes::Vector,
                           TwoJointTypes::Vector,
                           TwoJointTypes::Vector,
                           TwoJointTypes::Vector>(),
            pybind11::arg("torque") = TwoJointTypes::Vector::Zero(),
            pybind11::arg("position") = TwoJointTypes::Action::None(),
            pybind11::arg("position_kp") = TwoJointTypes::Action::None(),
            pybind11::arg("position_kd") = TwoJointTypes::Action::None());

    pybind11::class_<TwoJointTypes::Observation>(m, "Observation")
        .def_readwrite("position", &TwoJointTypes::Observation::position)
        .def_readwrite("velocity", &TwoJointTypes::Observation::velocity)
        .def_readwrite("torque", &TwoJointTypes::Observation::torque);

    pybind11::class_<TwoJointTypes::Frontend, TwoJointTypes::FrontendPtr>(m, "Frontend")
        .def(pybind11::init<robot_interfaces::TwoJointTypes::DataPtr>())
        .def("get_observation", &TwoJointTypes::Frontend::get_observation)
        .def("get_desired_action", &TwoJointTypes::Frontend::get_desired_action)
        .def("get_applied_action", &TwoJointTypes::Frontend::get_applied_action)
        .def("get_time_stamp_ms", &TwoJointTypes::Frontend::get_time_stamp_ms)
        .def("append_desired_action", &TwoJointTypes::Frontend::append_desired_action)
        .def("wait_until_time_index", &TwoJointTypes::Frontend::wait_until_timeindex)
        .def("get_current_time_index", &TwoJointTypes::Frontend::get_current_timeindex);
}
