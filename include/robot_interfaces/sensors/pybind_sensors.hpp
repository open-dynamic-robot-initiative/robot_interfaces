/**
 * @file
 * @brief Binds methods and objects to enable access from python
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <robot_interfaces/sensors/sensor_backend.hpp>
#include <robot_interfaces/sensors/sensor_data.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>
#include <robot_interfaces/sensors/sensor_frontend.hpp>

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
    // some typedefs to keep code below shorter
    typedef SensorData<ObservationType> BaseData;
    typedef SingleProcessSensorData<ObservationType> SingleProcData;
    typedef MultiProcessSensorData<ObservationType> MultiProcData;

    pybind11::class_<BaseData, std::shared_ptr<BaseData>>(m, "BaseData");

    pybind11::class_<SingleProcData,
                     std::shared_ptr<SingleProcData>,
                     BaseData>(m, "SingleProcessData")
        .def(pybind11::init<size_t>(), pybind11::arg("history_size") = 1000);

    pybind11::class_<MultiProcData,
                     std::shared_ptr<MultiProcData>,
                     BaseData>(m, "MultiProcessData")
        .def(pybind11::init<std::string, bool, size_t>(),
             pybind11::arg("shared_memory_id_prefix"),
             pybind11::arg("is_master"),
             pybind11::arg("history_size") = 1000);

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
