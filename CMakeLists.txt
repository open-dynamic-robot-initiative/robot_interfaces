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

# export the dependencies
ament_export_dependencies(pybind11 Eigen3 Boost real_time_tools time_series
                          signal_handler serialization_utils)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

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
target_link_libraries(${PROJECT_NAME} INTERFACE serialization_utils::gzip_iostream)
target_link_libraries(${PROJECT_NAME} INTERFACE Threads::Threads)
target_link_libraries(${PROJECT_NAME} INTERFACE pybind11::embed)
target_link_libraries(${PROJECT_NAME} INTERFACE Boost::iostreams)
# Export the target.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})


add_executable(convert_old_robot_log src/convert_old_robot_log.cpp)
target_link_libraries(convert_old_robot_log ${PROJECT_NAME})
list(APPEND all_targets convert_old_robot_log)

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
add_pybind11_module(py_generic srcpy/py_generic.cpp
    LINK_LIBRARIES ${PROJECT_NAME})
add_pybind11_module(py_finger_types srcpy/py_finger_types.cpp
    LINK_LIBRARIES ${PROJECT_NAME})
add_pybind11_module(py_trifinger_types srcpy/py_trifinger_types.cpp
    LINK_LIBRARIES ${PROJECT_NAME})
add_pybind11_module(py_one_joint_types srcpy/py_one_joint_types.cpp
    LINK_LIBRARIES ${PROJECT_NAME})
add_pybind11_module(py_two_joint_types srcpy/py_two_joint_types.cpp
    LINK_LIBRARIES ${PROJECT_NAME})
add_pybind11_module(py_solo_eight_types srcpy/py_solo_eight_types.cpp
    LINK_LIBRARIES ${PROJECT_NAME})

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
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

#
# Export the package as ament
#
ament_package()
