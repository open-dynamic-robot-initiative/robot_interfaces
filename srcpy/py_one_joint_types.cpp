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

#include <robot_interfaces/n_joint_robot_types.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_one_joint_types, m)
{
    pybind11::class_<robot_interfaces::NJointRobotTypes<1>::Data,
        robot_interfaces::NJointRobotTypes<1>::DataPtr>(m, "Data")
            .def(pybind11::init<>());

    pybind11::class_<robot_interfaces::NJointRobotTypes<1>::Backend,
        robot_interfaces::NJointRobotTypes<1>::BackendPtr>(m, "Backend")
            .def("initialize", &robot_interfaces::NJointRobotTypes<1>::Backend::initialize);

    pybind11::class_<NJointRobotTypes<1>::Action>(m, "Action")
        .def_readwrite("torque", &NJointRobotTypes<1>::Action::torque)
        .def_readwrite("position", &NJointRobotTypes<1>::Action::position)
        .def_readwrite("position_kp", &NJointRobotTypes<1>::Action::position_kp)
        .def_readwrite("position_kd", &NJointRobotTypes<1>::Action::position_kd)
        .def(
            pybind11::init<NJointRobotTypes<1>::Vector,
                           NJointRobotTypes<1>::Vector,
                           NJointRobotTypes<1>::Vector,
                           NJointRobotTypes<1>::Vector>(),
            pybind11::arg("torque") = NJointRobotTypes<1>::Vector::Zero(),
            pybind11::arg("position") = NJointRobotTypes<1>::Action::None(),
            pybind11::arg("position_kp") = NJointRobotTypes<1>::Action::None(),
            pybind11::arg("position_kd") = NJointRobotTypes<1>::Action::None());

    pybind11::class_<NJointRobotTypes<1>::Observation>(m, "Observation")
        .def_readwrite("position", &NJointRobotTypes<1>::Observation::position)
        .def_readwrite("velocity", &NJointRobotTypes<1>::Observation::velocity)
        .def_readwrite("torque", &NJointRobotTypes<1>::Observation::torque);

    pybind11::class_<NJointRobotTypes<1>::Frontend, NJointRobotTypes<1>::FrontendPtr>(m, "Frontend")
        .def(pybind11::init<robot_interfaces::NJointRobotTypes<1>::DataPtr>())
        .def("get_observation", &NJointRobotTypes<1>::Frontend::get_observation)
        .def("get_desired_action", &NJointRobotTypes<1>::Frontend::get_desired_action)
        .def("get_applied_action", &NJointRobotTypes<1>::Frontend::get_applied_action)
        .def("get_time_stamp_ms", &NJointRobotTypes<1>::Frontend::get_time_stamp_ms)
        .def("append_desired_action", &NJointRobotTypes<1>::Frontend::append_desired_action)
        .def("wait_until_time_index", &NJointRobotTypes<1>::Frontend::wait_until_timeindex)
        .def("get_current_time_index", &NJointRobotTypes<1>::Frontend::get_current_timeindex)
        .def("get_oldest_time_index", &NJointRobotTypes<1>::Frontend::get_oldest_timeindex);
}

