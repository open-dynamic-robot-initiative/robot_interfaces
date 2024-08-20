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
#include <robot_interfaces/utils.hpp>

namespace robot_interfaces
{
/**
 * @brief Create python bindings for different sensor types.
 *
 * @tparam The ObservationType
 */
template <typename ObservationType, typename InfoType = None>
void create_sensor_bindings(pybind11::module& m)
{
    pybind11::options options;
    // disable automatic function signature generation as this does not look too
    // nice in the Sphinx documentation.
    options.disable_function_signatures();

    // some typedefs to keep code below shorter
    typedef SensorData<ObservationType, InfoType> BaseData;
    typedef SingleProcessSensorData<ObservationType, InfoType> SingleProcData;
    typedef MultiProcessSensorData<ObservationType, InfoType> MultiProcData;
    typedef SensorLogger<ObservationType, InfoType> Logger;
    typedef SensorLogReader<ObservationType> LogReader;

    pybind11::class_<None, std::shared_ptr<None>>(
        m, "None", pybind11::module_local());

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

    pybind11::class_<SensorDriver<ObservationType, InfoType>,
                     std::shared_ptr<SensorDriver<ObservationType, InfoType>>>(
        m, "Driver");

    pybind11::class_<SensorBackend<ObservationType, InfoType>>(m, "Backend")
        .def(pybind11::init<
             typename std::shared_ptr<SensorDriver<ObservationType, InfoType>>,
             typename std::shared_ptr<BaseData>>())
        .def("shutdown",
             &SensorBackend<ObservationType, InfoType>::shutdown,
             pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<SensorFrontend<ObservationType, InfoType>>(m, "Frontend")
        .def(pybind11::init<typename std::shared_ptr<BaseData>>())
        .def("get_sensor_info",
             &SensorFrontend<ObservationType, InfoType>::get_sensor_info,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_latest_observation",
             &SensorFrontend<ObservationType, InfoType>::get_latest_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_observation",
             &SensorFrontend<ObservationType, InfoType>::get_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_timestamp_ms",
             &SensorFrontend<ObservationType, InfoType>::get_timestamp_ms,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_current_timeindex",
             &SensorFrontend<ObservationType, InfoType>::get_current_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<Logger, std::shared_ptr<Logger>>(m, "Logger")
        .def(pybind11::init<typename std::shared_ptr<BaseData>, size_t>())
        .def("start",
             &Logger::start,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("stop",
             &Logger::stop,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("reset",
             &Logger::reset,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("stop_and_save",
             &Logger::stop_and_save,
             pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<LogReader, std::shared_ptr<LogReader>>(m,
                                                            "LogReader",
                                                            R"XXX(
            LogReader(filename: str)

            See :meth:`read_file`
)XXX")
        .def(pybind11::init<std::string>())
        .def("read_file",
             &LogReader::read_file,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             pybind11::arg("filename"),
             R"XXX(
                read_file(filename: str)

                Read data from the specified camera log file.

                The data is stored in :attr:`data` and :attr:`timestamps`.

                Args:
                    filename (str): Path to the camera log file.
)XXX")
        .def_readonly("data",
                      &LogReader::data,
                      pybind11::call_guard<pybind11::gil_scoped_release>(),
                      "List of camera observations from the log file.")
        .def_readonly("timestamps",
                      &LogReader::timestamps,
                      pybind11::call_guard<pybind11::gil_scoped_release>(),
                      "List of timestamps of the camera observations.");
}

}  // namespace robot_interfaces
