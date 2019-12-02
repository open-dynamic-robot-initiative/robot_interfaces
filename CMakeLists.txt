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

)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

find_package(Pylon REQUIRED)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

# include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
# message(FATAL_ERROR "XXXXXXXXXXXXX ${PYLON_INCLUDE_DIR}")

search_for_eigen()

catkin_python_setup()


##set(Boost_USE_STATIC_LIBS OFF)
##set(Boost_USE_MULTITHREADED ON)
##set(Boost_USE_STATIC_RUNTIME OFF)
#find_package(Boost 1.45.0 REQUIRED)

#message(FATAL_ERROR ${Boost_INCLUDE_DIRS})


######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${PYLON_INCLUDE_DIRS}
#    ${Boost_INCLUDE_DIRS}
)

# link_directories(/opt/pylon/5/lib64)

########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${EXPORTED_LIBRAIRIES}
  CATKIN_DEPENDS ${CATKIN_PKGS}
)

####################
# manage the demos #
####################
add_subdirectory(demos)



#########################
# manage the unit tests #
#########################


###################
# python bindings #
###################

# camera #
set(py_camera_types_SRC_FILES
  srcpy/py_camera_types.cpp
)
pybind11_add_module(py_camera_types ${py_camera_types_SRC_FILES})
target_link_libraries(py_camera_types PRIVATE ${catkin_LIBRARIES})
set_target_properties(py_camera_types PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_camera_types DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

# finger #
set(py_finger_types_SRC_FILES
  srcpy/py_finger_types.cpp
)
pybind11_add_module(py_finger_types ${py_finger_types_SRC_FILES})
target_link_libraries(py_finger_types PRIVATE ${catkin_LIBRARIES})
set_target_properties(py_finger_types PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_finger_types DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

# one joint #
set(py_one_joint_types_SRC_FILES
  srcpy/py_one_joint_types.cpp
)
pybind11_add_module(py_one_joint_types ${py_one_joint_types_SRC_FILES})
target_link_libraries(py_one_joint_types PRIVATE ${catkin_LIBRARIES})
set_target_properties(py_one_joint_types PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_one_joint_types DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

# two joint #
set(py_two_joint_types_SRC_FILES
  srcpy/py_two_joint_types.cpp
)
pybind11_add_module(py_two_joint_types ${py_two_joint_types_SRC_FILES})
target_link_libraries(py_two_joint_types PRIVATE ${catkin_LIBRARIES})
set_target_properties(py_two_joint_types PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_two_joint_types DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


##########################
# building documentation #
##########################
build_doxygen_documentation()
