#include <robot_interfaces/pybind_camera.hpp>
#include <robot_interfaces/sensor_data_types.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_camera_types, m)
{
    create_camera_bindings<SensorDataTypes>(m);
}
