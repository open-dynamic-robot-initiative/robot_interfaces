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

using namespace robot_interfaces;

PYBIND11_MODULE(py_finger, m)
{

    pybind11::class_<Finger::Observation>(m, "Observation")
        .def_readwrite("angle", &Finger::Observation::angle)
        .def_readwrite("velocity", &Finger::Observation::velocity)
        .def_readwrite("torque", &Finger::Observation::torque);

    pybind11::class_<Finger, std::shared_ptr<Finger>>(m, "Finger")
        .def("get_observation", &Finger::get_observation)
        .def("get_desired_action", &Finger::get_desired_action)
        .def("get_applied_action", &Finger::get_applied_action)
        .def("get_time_stamp_ms", &Finger::get_time_stamp_ms)
        .def("append_desired_action", &Finger::append_desired_action)
        .def("wait_until_time_index", &Finger::wait_until_timeindex)
        .def("get_current_time_index", &Finger::get_current_timeindex);

}
