#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <robot_interfaces/camera_data.hpp>
#include <robot_interfaces/camera_frontend.hpp>
#include <robot_interfaces/camera_backend.hpp>
#include <robot_interfaces/camera_data_types.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_camera_types, m)
{
  pybind11::class_<typename CameraDataTypes::CData>(m, "CData")
          .def(pybind11::init<>());

  pybind11::class_<typename CameraDataTypes::CBackend>(m, "CBackend")
          .def(pybind11::init<>());

  pybind11::class_<typename CameraDataTypes::CFrontend>(m, "CFrontend")
      .def(pybind11::init<typename CameraDataTypes::CDataPtr>())
      .def("get_observation", &CameraDataTypes::CFrontend::get_observation)
      .def("get_time_stamp_ms", &CameraDataTypes::CFrontend::get_time_stamp_ms)
      .def("get_current_time_index", &CameraDataTypes::CFrontend::get_current_timeindex);

}
