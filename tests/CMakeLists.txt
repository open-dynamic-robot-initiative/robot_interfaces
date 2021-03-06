#
# Add unit tests.
#

find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

macro(create_unittest test_name)

  # create the executable
  ament_add_gtest(${test_name} main.cpp ${test_name}.cpp)
  target_link_libraries(${test_name} ${PROJECT_NAME} Boost::filesystem)

endmacro(create_unittest test_name)

create_unittest(test_robot_backend)
create_unittest(test_robot_logger)
create_unittest(test_sensor_interface)
create_unittest(test_sensor_logger)

# Python tests
ament_add_pytest_test(test_pybind_pickle_py test_pybind_pickle.py)
