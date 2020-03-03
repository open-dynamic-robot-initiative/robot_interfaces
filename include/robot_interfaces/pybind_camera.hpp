#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <robot_interfaces/camera_data.hpp>
#include <robot_interfaces/camera_frontend.hpp>
#include <robot_interfaces/camera_backend.hpp>
#include <robot_interfaces/camera_data_types.hpp>

namespace robot_interfaces
{

template <typename Types>
void create_camera_bindings(pybind11::module &m)
{
    pybind11::class_<typename Types::CData,
        typename Types::CDataPtr>(m, "CData")
          .def(pybind11::init<>());

    pybind11::class_<typename Types::CBackend>(m, "CBackend")
          .def(pybind11::init<typename Types::CDriverPtr,typename Types::CDataPtr>());

    pybind11::class_<typename Types::CDriver>(m, "CDriver")
          .def(pybind11::init<>());

    pybind11::class_<typename Types::CFrontend>(m, "CFrontend")
      .def(pybind11::init<typename Types::CDataPtr>())
      .def("get_observation", &Types::CFrontend::get_observation)
      .def("get_time_stamp_ms", &Types::CFrontend::get_time_stamp_ms)
      .def("get_current_time_index", &Types::CFrontend::get_current_timeindex);

}

}



// <CameraDataTypes<720,540>>
