/**
 * @file
 * @brief Create bindings for three pylon dependent camera sensors
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#include <robot_interfaces/sensors/tricamera_observation.hpp>
#ifdef Pylon_FOUND
#include <robot_interfaces/sensors/tricamera_driver.hpp>
#endif

#include <robot_interfaces/sensors/pybind_sensors.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_tricamera_types, m)
{
    create_sensor_bindings<TricameraObservation>(m);

#ifdef Pylon_FOUND
    pybind11::class_<TriCameraDriver,
                     std::shared_ptr<TriCameraDriver>,
                     SensorDriver<TricameraObservation>>(m, "TriCameraDriver")
        .def(pybind11::init<const std::string&,
                            const std::string&,
                            const std::string&>())
        .def("get_observation", &TriCameraDriver::get_observation);
#endif

    pybind11::class_<TriCameraObservation>(m, "TriCameraObservation")
        .def(pybind11::init<>())
        .def_readwrite("cameras", &TriCameraObservation::cameras);
}
