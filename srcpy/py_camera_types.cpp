#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <robot_interfaces/camera_data.hpp>
#include <robot_interfaces/camera_frontend.hpp>
#include <robot_interfaces/camera_backend.hpp>
#include <robot_interfaces/camera_data_types.hpp>

using namespace robot_interfaces;

template <typename <CameraDataTypes<720,540>>>
PYBIND11_MODULE(py_camera_types, m)
{
  pybind11::class_<typename <CameraDataTypes<720,540>>>::CData>(m, "CData")
          .def(pybind11::init<>());

  pybind11::class_<typename <CameraDataTypes<720,540>>>::CBackend>(m, "CBackend")
          .def(pybind11::init<>());

  pybind11::class_<typename <CameraDataTypes<720,540>>::CFrontend>(m, "CFrontend")
      .def(pybind11::init<typename <CameraDataTypes<720,540>> >::CDataPtr>())
      .def("get_observation", &<CameraDataTypes<720,540>>::CFrontend::get_observation)
      .def("get_time_stamp_ms", &<CameraDataTypes<720,540>>::CFrontend::get_time_stamp_ms)
      .def("get_current_time_index", &<CameraDataTypes<720,540>>::CFrontend::get_current_timeindex);

}
