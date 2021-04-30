Build Instructions
==================


.. note::

    If you intend to use this interface to control your own robot, this package
    (and its dependencies) is enough, and you can follow the instructions below.
    If you are looking for the interface of the TriFinger robot interface, see
    the installation instructions of the robot_fingers_ package instead (this
    also includes ``robot_interfaces``).


Dependencies
------------

We are using colcon_ as build tool and typically use `ros2 run` to execute our
applications.  While we are not really depending on any ROS_ packages, this
means a basic ROS 2 installation is recommended.

We are testing on Ubuntu 20.04 with ROS Foxy.  Other versions may work as well
but are not officially supported.

.. note::

    We provide a Singularity image with all dependencies for the TriFinger robot
    which also covers everything needed for ``robot_interfaces``.  See the
    documentation of the robot_fingers_ package for more information.


Get the Source
--------------

``robot_interfaces`` depends on several other of our packages which are
organized in separate repositories.  We therefore use the management tool treep_
which allows easy cloning of multi-repository projects.

treep can be installed via pip::

    pip install treep

Clone the treep configuration containing the "ROBOT_INTERFACES" project::

    git clone git@github.com:machines-in-motion/treep_machines_in_motion.git

.. note::

    treep searches for a configuration directory from the current working
    directory upwards.  So you can use treep in the directory in which you
    invoked the ``git clone`` command above or any subdirectory.

Now clone the project::

    treep --clone ROBOT_INTERFACES

.. important::

    treep uses SSH to clone from github.  So for the above command to work, you
    need a github account with a registered SSH key.  Further this key needs to
    work without asking for a password everytime.  To achieve this, run

    ::

        ssh-add

    first.

You should now have the following directory structure::

    ├── treep_machines_in_motion
    └── workspace
        └── src
            ├── googletest
            ├── mpi_cmake_modules
            ├── package_template
            ├── pybind11
            ├── real_time_tools
            ├── robot_interfaces
            ├── serialization_utils
            ├── shared_memory
            ├── signal_handler
            ├── time_series
            └── yaml_utils


Build
-----

To build, cd into the ``workspace`` directory and call

::

    colcon build

to build the whole workspace.


.. _robot_fingers: http://people.tuebingen.mpg.de/mpi-is-software/robotfingers/docs/robot_fingers/
.. _colcon: https://colcon.readthedocs.io/en/released/index.html
.. _ROS: https://www.ros.org
.. _treep: https://pypi.org/project/treep/
