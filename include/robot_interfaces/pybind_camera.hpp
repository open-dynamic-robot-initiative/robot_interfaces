#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <opencv2/opencv.hpp>
#include <robot_interfaces/sensor_backend.hpp>
#include <robot_interfaces/sensor_data.hpp>
#include <robot_interfaces/sensor_data_types.hpp>
#include <robot_interfaces/sensor_frontend.hpp>

namespace robot_interfaces
{
/**
 * @brief Create python bindings for different camera types.
 *
 * @tparam An instance of SensorDataTypes
 */

template <typename Types>
void create_camera_bindings(pybind11::module& m)
{
    pybind11::class_<typename Types::Data, typename Types::DataPtr>(
        m, "SensorData")
        .def(pybind11::init<>());

    pybind11::class_<typename Types::Backend>(m, "SensorBackend")
        .def(pybind11::init<typename Types::CVDriverPtr,
                            typename Types::DataPtr>());

    pybind11::class_<typename Types::CVDriver, typename Types::CVDriverPtr>(
        m, "OpenCVDriver")
        .def(pybind11::init<>())
        .def("pylon_init", &Types::CVDriver::pylon_init)
        .def("grab_frame", &Types::CVDriver::grab_frame);

    pybind11::class_<typename Types::Frontend>(m, "SensorFrontend")
        .def(pybind11::init<typename Types::DataPtr>())
        .def("get_latest_observation", &Types::Frontend::get_latest_observation)
        .def("get_observation", &Types::Frontend::get_observation)
        .def("get_timestamp_ms", &Types::Frontend::get_timestamp_ms)
        .def("get_current_timeindex", &Types::Frontend::get_current_timeindex);

    pybind11::class_<typename Types::OpenCVObservation>(m, "OpenCVObservation")
        .def(pybind11::init<>())
        .def_readwrite("image", &Types::OpenCVObservation::image)
        .def_readwrite("time_stamp", &Types::OpenCVObservation::time_stamp);

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
}  // namespace robot_interfaces
