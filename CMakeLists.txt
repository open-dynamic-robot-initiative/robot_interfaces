#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# Set up the project.
#
cmake_minimum_required(VERSION 3.10.2)

project(robot_interfaces)

# Using C++17.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies.
#
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(time_series REQUIRED)
find_package(signal_handler REQUIRED)
find_package(serialization_utils REQUIRED)
find_package(Threads)
find_package(rt)

# export de dependencies
ament_export_dependencies(pybind11 Eigen3 real_time_tools time_series
                          signal_handler serialization_utils)

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)

#
# Add main target.
#
add_library(${PROJECT_NAME} INTERFACE)
# Add the include dependencies
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
# Link the dependencies
target_link_libraries(${PROJECT_NAME} INTERFACE rt::rt)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} INTERFACE real_time_tools::real_time_tools)
target_link_libraries(${PROJECT_NAME} INTERFACE time_series::time_series)
target_link_libraries(${PROJECT_NAME} INTERFACE signal_handler::signal_handler)
target_link_libraries(${PROJECT_NAME} INTERFACE serialization_utils::serialization_utils)
target_link_libraries(${PROJECT_NAME} INTERFACE Threads::Threads)
target_link_libraries(${PROJECT_NAME} INTERFACE pybind11::embed)
# Export the target.
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})

#
# manage the demos.
#
add_executable(demo demos/demo.cpp)
target_link_libraries(demo ${PROJECT_NAME})
list(APPEND all_targets demo)

add_executable(demo_multiprocess_backend demos/demo_multiprocess_backend.cpp)
target_link_libraries(demo_multiprocess_backend ${PROJECT_NAME})
list(APPEND all_targets demo_multiprocess_backend)

add_executable(demo_multiprocess_frontend demos/demo_multiprocess_frontend.cpp)
target_link_libraries(demo_multiprocess_frontend ${PROJECT_NAME})
list(APPEND all_targets demo_multiprocess_frontend)

#
# manage the unit tests.
#
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

#
# python bindings.
#

macro(ADD_PYTHON_BINDINGS module_name)
  add_library(${module_name}_cpp_bindings MODULE srcpy/${module_name}.cpp)
  target_link_libraries(${module_name}_cpp_bindings PRIVATE pybind11::module)
  target_link_libraries(${module_name}_cpp_bindings PRIVATE ${PYTHON_LIBRARIES})
  target_link_libraries(${module_name}_cpp_bindings PRIVATE ${PROJECT_NAME})
  set_target_properties(
    ${module_name}_cpp_bindings
    PROPERTIES OUTPUT_NAME ${module_name}
               PREFIX ""
               SUFFIX "${PYTHON_MODULE_EXTENSION}")
  target_include_directories(
    ${module_name}_cpp_bindings
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include> SYSTEM
    PUBLIC ${PYTHON_INCLUDE_DIRS})
  _ament_cmake_python_get_python_install_dir()
  install(TARGETS ${module_name}_cpp_bindings DESTINATION ${PYTHON_INSTALL_DIR})
endmacro(ADD_PYTHON_BINDINGS module_name)

add_python_bindings(py_generic)
add_python_bindings(py_finger_types)
add_python_bindings(py_trifinger_types)
add_python_bindings(py_one_joint_types)
add_python_bindings(py_two_joint_types)

#
# building documentation
#
add_documentation()

#
# Install the package
#
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

#
# Export the package as ament
#
ament_package()
