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
#include <robot_interfaces/sensors/sensor_log_reader.hpp>
#include <robot_interfaces/sensors/sensor_logger.hpp>

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
    typedef SensorLogger<ObservationType> Logger;
    typedef SensorLogReader<ObservationType> LogReader;

    pybind11::class_<BaseData, std::shared_ptr<BaseData>>(m, "BaseData");

    pybind11::class_<SingleProcData, std::shared_ptr<SingleProcData>, BaseData>(
        m, "SingleProcessData")
        .def(pybind11::init<size_t>(), pybind11::arg("history_size") = 1000);

    pybind11::class_<MultiProcData, std::shared_ptr<MultiProcData>, BaseData>(
        m, "MultiProcessData")
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
             typename std::shared_ptr<BaseData>>());

    pybind11::class_<SensorFrontend<ObservationType>>(m, "Frontend")
        .def(pybind11::init<typename std::shared_ptr<BaseData>>())
        .def("get_latest_observation",
             &SensorFrontend<ObservationType>::get_latest_observation)
        .def("get_observation",
             &SensorFrontend<ObservationType>::get_observation)
        .def("get_timestamp_ms",
             &SensorFrontend<ObservationType>::get_timestamp_ms)
        .def("get_current_timeindex",
             &SensorFrontend<ObservationType>::get_current_timeindex);

    pybind11::class_<Logger, std::shared_ptr<Logger>>(m, "Logger")
        .def(pybind11::init<typename std::shared_ptr<BaseData>, size_t>())
        .def("start", &Logger::start)
        .def("stop", &Logger::stop)
        .def("reset", &Logger::reset)
        .def("stop_and_save", &Logger::stop_and_save);

    pybind11::class_<LogReader, std::shared_ptr<LogReader>>(m, "LogReader")
        .def(pybind11::init<std::string>())
        .def("read_file", &LogReader::read_file)
        .def_readonly("data", &LogReader::data)
        .def_readonly("timestamps", &LogReader::timestamps);
}

}  // namespace robot_interfaces
