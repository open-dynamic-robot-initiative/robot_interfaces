/**
 * @file
 * @brief Create bindings for camera sensors
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#include <robot_interfaces/sensors/tricamera_observation.hpp>
#ifdef Pylon_FOUND
#include <robot_interfaces/sensors/sync_driver.hpp>
#endif

#include <robot_interfaces/sensors/pybind_sensors.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_tricamera_types, m)
{
    create_sensor_bindings<TricameraObservation>(m);

#ifdef Pylon_FOUND
    pybind11::class_<SyncDriver, std::shared_ptr<SyncDriver>,
                     SensorDriver<TricameraObservation>>(m, "SyncDriver")
        .def(pybind11::init<const std::string&, const std::string&, const std::string&>())
        .def("get_observation", &SyncDriver::get_observation);     
#endif


    pybind11::class_<TricameraObservation>(m, "TricameraObservation")
        .def(pybind11::init<>())
        .def_readwrite("cam_array", &TricameraObservation::cam_array);
}