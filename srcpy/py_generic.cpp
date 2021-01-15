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
 * \brief Create bindings for generic types
 */
#include <robot_interfaces/pybind_helper.hpp>
#include <robot_interfaces/status.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_generic, m)
{
    pybind11::class_<Status> pystatus(m, "Status");
    pystatus.def(pybind11::init<>())
        .def_readwrite(
            "action_repetitions",
            &Status::action_repetitions,
            "int: Number of times the current action has been repeated.")
        .def_readonly("error_status",
                      &Status::error_status,
                      "ErrorStatus: Current error status.")
        .def("set_error",
             &Status::set_error)
        .def("get_error_message",
             &Status::get_error_message);

    pybind11::enum_<Status::ErrorStatus>(pystatus, "ErrorStatus")
        .value("NO_ERROR",
               Status::ErrorStatus::NO_ERROR,
               "Indicates that there is no error.")
        .value("DRIVER_ERROR",
               Status::ErrorStatus::DRIVER_ERROR,
               "Error from the low level robot driver (e.g. some hardware "
               "failure).")
        .value(
            "BACKEND_ERROR",
            Status::ErrorStatus::BACKEND_ERROR,
            "Error from the robot back end (e.g. some communication issue).");
}
