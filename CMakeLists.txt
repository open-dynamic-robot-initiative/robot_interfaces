######################
# set up the project #
######################
cmake_minimum_required(VERSION 3.1)

project(robot_interfaces)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# stop build on first error
string(APPEND CMAKE_CXX_FLAGS " -Wfatal-errors")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################
set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  real_time_tools
  pybind11_catkin
  time_series
  signal_handler
  serialization_utils
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_eigen()

catkin_python_setup()


######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_PKGS}
)


####################
# manage the demos #
####################
add_executable(demo demos/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES})

add_executable(demo_multiprocess_backend demos/demo_multiprocess_backend.cpp)
target_link_libraries(demo_multiprocess_backend ${catkin_LIBRARIES}
    rt pthread)

add_executable(demo_multiprocess_frontend demos/demo_multiprocess_frontend.cpp)
target_link_libraries(demo_multiprocess_frontend ${catkin_LIBRARIES}
    rt pthread)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

###################
# python bindings #
###################

catkin_add_pybind11_module(py_generic)
catkin_add_pybind11_module(py_finger_types)
catkin_add_pybind11_module(py_trifinger_types)
catkin_add_pybind11_module(py_one_joint_types)
catkin_add_pybind11_module(py_two_joint_types)


##########################
# building documentation #
##########################
add_doxygen_documentation()
