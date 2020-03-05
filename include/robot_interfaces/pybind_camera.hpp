#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <robot_interfaces/sensor_data.hpp>
#include <robot_interfaces/sensor_frontend.hpp>
#include <robot_interfaces/sensor_backend.hpp>
#include <robot_interfaces/sensor_data_types.hpp>

namespace robot_interfaces
{

template <typename Types>
void create_camera_bindings(pybind11::module &m)
{
    pybind11::class_<typename Types::Data,
        typename Types::DataPtr>(m, "SensorData")
          .def(pybind11::init<>());

    pybind11::class_<typename Types::Backend>(m, "SensorBackend")
          .def(pybind11::init<typename Types::CVDriverPtr, typename Types::DataPtr>());

    pybind11::class_<typename Types::CVDriver, typename Types::CVDriverPtr>(m, "OpenCVDriver")
          .def(pybind11::init<>());
    pybind11::class_<typename Types::Frontend>(m, "SensorFrontend")
      .def(pybind11::init<typename Types::DataPtr>())
      .def("get_observation", &Types::Frontend::get_observation)
      .def("get_timestamp_ms", &Types::Frontend::get_time_stamp_ms)
      .def("get_current_timeindex", &Types::Frontend::get_current_timeindex);

//     PYBIND11_DECLARE_HOLDER_TYPE(CData, std::shared_ptr<CData>);  

}

}



// <CameraDataTypes<720,540>>
