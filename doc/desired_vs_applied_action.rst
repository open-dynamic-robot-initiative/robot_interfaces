Desired vs Applied Action
=========================

The action given by the user is called the *desired* action.  Depending on the
implementation of the :cpp:class:`~robot_interfaces::RobotDriver`, this action
may be altered before it is actually applied on the robot, e.g. by some safety
checks limiting torque and maximum velocity.  This altered action is called the
*applied* action.  You can use
:cpp:func:`robot_interfaces::RobotFrontend::get_applied_action` to see what
action actually got applied on the robot.
