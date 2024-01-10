RobotData -- Single or Multi Process
====================================

The *RobotData* serves as a communication link between the back end and the
front end (see :ref:`architecture`).  It contains time series structures, in
which all the data (actions, observations, etc.) is stored.

The interface is defined in the base class
:cpp:class:`~robot_interfaces::RobotData`.  There are two different
implementations that implement the interface:

- A "single-process" implementation
  (:cpp:class:`~robot_interfaces::SingleProcessRobotData`), keeping the data in
  a buffer in normal memory. This allows fastest access but all modules (front
  end, back end, ...) must run in the same process.
  This is easiest to use for simple setups where everything is in one script.
- A "multi-process" implementation
  (:cpp:class:`~robot_interfaces::MultiProcessRobotData`) using a `shared memory
  <https://github.com/machines-in-motion/shared_memory>`_ which can be accessed
  by multiple processes. With this, front end, back end, etc. can be executed as
  separate processes. This allows a more secure setup as user code (using the
  front end) and back-end configuration can be decoupled, thus not allowing the
  user to change safety-critical settings of the robot (e.g. the maximum motor
  current).

See the demos_ for implementations with both the single- and the multi-process
RobotData.


.. _demos: https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos
