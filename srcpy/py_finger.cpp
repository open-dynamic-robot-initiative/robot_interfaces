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

#include <robot_interfaces/finger.hpp>

using namespace robot_interfaces::finger;

PYBIND11_MODULE(py_finger, m)
{

  //pybind11::class_<robot_interfaces::finger::Data,
  //    robot_interfaces::finger::DataPtr>(m, "Data");
    pybind11::class_<Data, DataPtr>(m, "Data")
            .def(pybind11::init<>());

    //pybind11::class_<robot_interfaces::finger::Backend,
    //    robot_interfaces::finger::BackendPtr>(m, "Backend");
    pybind11::class_<Backend, BackendPtr>(m, "Backend");

    pybind11::class_<Observation>(m, "Observation")
        .def_readwrite("angle", &Observation::angle)
        .def_readwrite("velocity", &Observation::velocity)
        .def_readwrite("torque", &Observation::torque);

    pybind11::class_<Frontend, FrontendPtr>(m, "Frontend")
        .def(pybind11::init<robot_interfaces::finger::DataPtr>())
        .def("get_observation", &Frontend::get_observation)
        .def("get_desired_action", &Frontend::get_desired_action)
        .def("get_applied_action", &Frontend::get_applied_action)
        .def("get_time_stamp_ms", &Frontend::get_time_stamp_ms)
        .def("append_desired_action", &Frontend::append_desired_action)
        .def("wait_until_time_index", &Frontend::wait_until_timeindex)
        .def("get_current_time_index", &Frontend::get_current_timeindex);

}
