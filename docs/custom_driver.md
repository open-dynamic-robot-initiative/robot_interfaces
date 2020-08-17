How to Implement a Custom RobotDriver
=====================================

To use robot_interfaces with your own robot, you need to provide implementations
for

- the Action type
- the Observation type
- the RobotDriver


Action and Observation
----------------------

Action and observation can be any arbitrary type, whatever is needed for your
robot.  There are only the following restrictions:

- If you want to use the @ref robot_interfaces::MultiProcessRobotData, the
  action and observation types need to be serializable with
  [cereal](https://uscilab.github.io/cereal/).
- If you want to use the @ref robot_interfaces::RobotLogger, the action and
  observation types need to inherit from @ref robot_interfaces::Loggable.


RobotDriver
-----------

Your robot driver needs to be a class that inherits from @ref
robot_interfaces::RobotDriver, using your `Action` and `Observation` types (see
above).


Example
-------

See the example implementation of a dummy robot driver in [example.hpp](https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/include/robot_interfaces/example.hpp).  This dummy driver is also used in the [demos](https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos).
