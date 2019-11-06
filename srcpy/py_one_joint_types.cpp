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

#include <robot_interfaces/one_joint_types.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_one_joint_types, m)
{
    pybind11::class_<robot_interfaces::OneJointTypes::Data,
        robot_interfaces::OneJointTypes::DataPtr>(m, "Data")
            .def(pybind11::init<>());

    pybind11::class_<robot_interfaces::OneJointTypes::Backend,
        robot_interfaces::OneJointTypes::BackendPtr>(m, "Backend")
            .def("initialize", &robot_interfaces::OneJointTypes::Backend::initialize);

    pybind11::class_<OneJointTypes::Action>(m, "Action")
        .def_readwrite("torque", &OneJointTypes::Action::torque)
        .def_readwrite("position", &OneJointTypes::Action::position)
        .def_readwrite("position_kp", &OneJointTypes::Action::position_kp)
        .def_readwrite("position_kd", &OneJointTypes::Action::position_kd)
        .def(
            pybind11::init<OneJointTypes::Vector,
                           OneJointTypes::Vector,
                           OneJointTypes::Vector,
                           OneJointTypes::Vector>(),
            pybind11::arg("torque") = OneJointTypes::Vector::Zero(),
            pybind11::arg("position") = OneJointTypes::Action::None(),
            pybind11::arg("position_kp") = OneJointTypes::Action::None(),
            pybind11::arg("position_kd") = OneJointTypes::Action::None());

    pybind11::class_<OneJointTypes::Observation>(m, "Observation")
        .def_readwrite("position", &OneJointTypes::Observation::position)
        .def_readwrite("velocity", &OneJointTypes::Observation::velocity)
        .def_readwrite("torque", &OneJointTypes::Observation::torque);

    pybind11::class_<OneJointTypes::Frontend, OneJointTypes::FrontendPtr>(m, "Frontend")
        .def(pybind11::init<robot_interfaces::OneJointTypes::DataPtr>())
        .def("get_observation", &OneJointTypes::Frontend::get_observation)
        .def("get_desired_action", &OneJointTypes::Frontend::get_desired_action)
        .def("get_applied_action", &OneJointTypes::Frontend::get_applied_action)
        .def("get_time_stamp_ms", &OneJointTypes::Frontend::get_time_stamp_ms)
        .def("append_desired_action", &OneJointTypes::Frontend::append_desired_action)
        .def("wait_until_time_index", &OneJointTypes::Frontend::wait_until_timeindex)
        .def("get_current_time_index", &OneJointTypes::Frontend::get_current_timeindex);
}

