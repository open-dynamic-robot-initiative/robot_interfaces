Build Instructions
==================


@note If you intend to use this interface to control your own robot, this
package is enough.  If you are looking for the interface of the TriFinger robot
interface, see the installation instructions of the [`robot_fingers`
package](https://open-dynamic-robot-initiative.github.io/code_documentation/robot_fingers/docs/doxygen/html/index.html)
instead (this also includes `robot_interfaces`).

Dependencies
------------

We are using [catkin](http://wiki.ros.org/catkin) as build tool (i.e.
`robot_interfaces` is a catkin package).  While we are not really depending on
any [ROS](http://www.ros.org) packages, this means you need a basic ROS
installation to build.

In the following we are using catkin_tools which need to be installed
separately:

    pip install catkin_tools


@note We provide a Singularity image with all dependencies for the TriFinger
robot which also covers everything needed for `robot_interfaces`.  See the
documentation of the [`robot_fingers`
package](https://open-dynamic-robot-initiative.github.io/code_documentation/robot_fingers/docs/doxygen/html/index.html)
for more information.


Get the Source
--------------

`robot_interfaces` depends on several other of our packages which are
organized in separate repositories.  We therefore use a workspace management
tool called [treep](https://pypi.org/project/treep/) which allows easy cloning
of multi-repository projects.

treep can be installed via pip:

    pip install treep

Clone the treep configuration containing the "ROBOT_INTERFACES" project:

    git clone git@github.com:machines-in-motion/treep_machines_in_motion.git

@note  treep searches for a configuration directory from the current working
directory upwards.  So you can use treep in the directory in which you invoked
the `git clone` command above or any subdirectory.

Now clone the project:

    treep --clone ROBOT_INTERFACES

@note **Important:** treep uses SSH to clone from github.  So for the above command to
work, you need a github account with a registered SSH key.  Further this key
needs to work without asking for a password everytime.  To achieve this, run

    ssh-add

first.

You should now have the following directory structure:

    ├── treep_machines_in_motion
    └── workspace
        └── src
            ├── catkin
            │   ├── core_robotics
            │   │   ├── mpi_cmake_modules
            │   │   ├── pybind11_catkin
            │   │   ├── real_time_tools
            │   │   ├── shared_memory
            │   │   ├── time_series
            │   │   └── yaml_cpp_catkin
            │   ├── examples
            │   │   └── ci_example
            │   ├── robots
            │   │   └── robot_interfaces
            │   └── tools
            │       ├── serialization_utils
            │       └── signal_handler
            └── not_catkin
                └── third_party
                    └── pybind11


Build
-----

To build, cd into the `workspace` directory and call

    catkin build

to build the whole workspace.

### Python Bindings

With the above command Python bindings will be build for the default python
version of your system (see `python --version`).  If you want to use a
different version (e.g. python3), you can specify as follows:

    catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3

