######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(robot_interfaces)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

############################
# Search for dependencies. #
############################
set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  real_time_tools
  mpi_cpp_tools
  pybind11_catkin
  time_series
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})
find_package(OpenCV REQUIRED)
find_package(Pylon)

if (${Pylon_FOUND})
    add_definitions(-DPylon_FOUND)
endif()

search_for_eigen()

catkin_python_setup()


######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Pylon_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_PKGS}
  LIBRARIES global_signal_handler
)


add_library(global_signal_handler src/global_signal_handler.cpp)
target_link_libraries(global_signal_handler ${catkin_LIBRARIES})

####################
# manage the demos #
####################

# to be replaced by a unit test
add_executable(demo_infinite_wait demos/demo_infinite_wait.cpp)
target_link_libraries(demo_infinite_wait ${catkin_LIBRARIES} global_signal_handler)

add_executable(demo demos/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} global_signal_handler)

add_executable(demo_multiprocess_backend demos/demo_multiprocess_backend.cpp)
target_link_libraries(demo_multiprocess_backend ${catkin_LIBRARIES}
    global_signal_handler rt pthread)

add_executable(demo_multiprocess_frontend demos/demo_multiprocess_frontend.cpp)
target_link_libraries(demo_multiprocess_frontend ${catkin_LIBRARIES}
    global_signal_handler rt pthread)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

###################
# python bindings #
###################

macro(add_python_bindings module_name)
    pybind11_add_module(${module_name} srcpy/${module_name}.cpp)
    target_link_libraries(${module_name} PRIVATE
        ${catkin_LIBRARIES} global_signal_handler ${ARGN})
    set_target_properties(${module_name} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
    install(TARGETS ${module_name} DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
endmacro()

add_python_bindings(py_generic)
add_python_bindings(py_camera_types ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES})
add_python_bindings(py_finger_types)
add_python_bindings(py_trifinger_types)
add_python_bindings(py_one_joint_types)
add_python_bindings(py_two_joint_types)


##########################
# building documentation #
##########################
build_doxygen_documentation()
