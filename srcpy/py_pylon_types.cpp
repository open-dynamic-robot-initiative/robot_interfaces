/**
 * @file
 * @brief Create bindings for camera sensors
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#include <robot_interfaces/sensors/camera_observation.hpp>
#include <robot_interfaces/sensors/pylon_driver.hpp>
#include <robot_interfaces/sensors/pybind_sensors.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_pylon_types, m)
{
    create_sensor_bindings<CameraObservation>(m);

    pybind11::class_<PylonDriver,
                     std::shared_ptr<PylonDriver>,
                     SensorDriver<CameraObservation>>(m, "PylonDriver")
        .def(pybind11::init<>())
        .def("is_access_successful", &PylonDriver::is_access_successful)
        .def("get_observation", &PylonDriver::get_observation);

    pybind11::class_<CameraObservation>(m, "CameraObservation")
        .def(pybind11::init<>())
        .def_readwrite("image", &CameraObservation::image)
        .def_readwrite("time_stamp", &CameraObservation::time_stamp);

    // The following block of code for binding cv::Mat to np.ndarray is from
    // [here](https://alexsm.com/pybind11-buffer-protocol-opencv-to-numpy/).
    pybind11::class_<cv::Mat>(m, "Image", pybind11::buffer_protocol())
        .def_buffer([](cv::Mat& im) -> pybind11::buffer_info {
            return pybind11::buffer_info(
                // Pointer to buffer
                im.data,
                // Size of one scalar
                sizeof(uint8_t),
                // Python struct-style format descriptor
                pybind11::format_descriptor<uint8_t>::format(),
                // Number of dimensions
                3,
                // Buffer dimensions
                {im.rows, im.cols, im.channels()},
                // Strides (in bytes) for each index
                {sizeof(uint8_t) * im.channels() * im.cols,
                 sizeof(uint8_t) * im.channels(),
                 sizeof(uint8_t)});
        });
}