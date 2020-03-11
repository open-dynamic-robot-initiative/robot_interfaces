/**
 * @file
 * @brief Binds methods and objects to enable access from python
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <opencv2/opencv.hpp>
#include <robot_interfaces/sensor_backend.hpp>
#include <robot_interfaces/sensor_data.hpp>
#include <robot_interfaces/sensor_driver.hpp>
#include <robot_interfaces/sensor_frontend.hpp>

namespace robot_interfaces
{
/**
 * @brief Create python bindings for different sensor types.
 *
 * @tparam The ObservationType
 */

template <typename ObservationType>
void create_sensor_bindings(pybind11::module& m)
{
    pybind11::class_<SensorData<ObservationType>,
                     std::shared_ptr<SensorData<ObservationType>>>(m, "Data")
        .def(pybind11::init<>());

    pybind11::class_<SensorDriver<ObservationType>,
                     std::shared_ptr<SensorDriver<ObservationType>>>(m,
                                                                     "Driver");

    pybind11::class_<SensorBackend<ObservationType>>(m, "Backend")
        .def(pybind11::init<
             typename std::shared_ptr<SensorDriver<ObservationType>>,
             typename std::shared_ptr<SensorData<ObservationType>>>());

    pybind11::class_<SensorFrontend<ObservationType>>(m, "Frontend")
        .def(pybind11::init<
             typename std::shared_ptr<SensorData<ObservationType>>>())
        .def("get_latest_observation",
             &SensorFrontend<ObservationType>::get_latest_observation)
        .def("get_observation",
             &SensorFrontend<ObservationType>::get_observation)
        .def("get_timestamp_ms",
             &SensorFrontend<ObservationType>::get_timestamp_ms)
        .def("get_current_timeindex",
             &SensorFrontend<ObservationType>::get_current_timeindex);
}
}  // namespace robot_interfaces
