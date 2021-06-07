/**
 * @file
 * @brief Create Python bindings for generic types
 * @copyright 2019, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#include <time_series/pybind11_helper.hpp>

#include <robot_interfaces/pybind_helper.hpp>
#include <robot_interfaces/robot_backend.hpp>
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
        .def("set_error", &Status::set_error)
        .def("get_error_message", &Status::get_error_message)
        .def(pybind11::pickle(
            [](const Status &status) {  // __getstate__
                return pybind11::make_tuple(status.action_repetitions,
                                            status.error_status,
                                            status.get_error_message());
            },
            [](pybind11::tuple t) {  // __setstate__
                if (t.size() != 3)
                {
                    throw std::runtime_error("Invalid state!");
                }

                Status status;
                status.action_repetitions = t[0].cast<int>();
                status.set_error(t[1].cast<Status::ErrorStatus>(),
                                 t[2].cast<std::string>());

                return status;
            }));

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

    time_series::create_python_bindings<Status>(m, "_StatusTimeSeries");
    time_series::create_multiprocesses_python_bindings<Status>(
        m, "_StatusMultiProcessTimeSeries");

    pybind11::enum_<RobotBackendTerminationReason>(
        m, "RobotBackendTerminationReason")
        .value("NOT_TERMINATED", RobotBackendTerminationReason::NOT_TERMINATED)
        .value("SHUTDOWN_REQUESTED",
               RobotBackendTerminationReason::SHUTDOWN_REQUESTED)
        .value("MAXIMUM_NUMBER_OF_ACTIONS_REACHED",
               RobotBackendTerminationReason::MAXIMUM_NUMBER_OF_ACTIONS_REACHED)
        .value("DRIVER_ERROR", RobotBackendTerminationReason::DRIVER_ERROR)
        .value("FIRST_ACTION_TIMEOUT",
               RobotBackendTerminationReason::FIRST_ACTION_TIMEOUT)
        .value("NEXT_ACTION_TIMEOUT",
               RobotBackendTerminationReason::NEXT_ACTION_TIMEOUT);
}
