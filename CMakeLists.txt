######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(robot_interfaces)

# required to use std::shared_ptr in code
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

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
mpi_cpp_tools
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_eigen()


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
#    ${Boost_INCLUDE_DIRS}
)

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




##########################
# building documentation #
##########################
build_doxygen_documentation()