RobotData -- Single or Multi Process
====================================


The [RobotData](@ref robot_interfaces::RobotData) class serves as a
communication link between the back end and the front end.  All data is stored
there in time series.

There are two different implementations of `RobotData`:

- [SingleProcessRobotData](@ref robot_interfaces::SingleProcessRobotData):  Uses
  normal memory for the time series.  Use this if all modules (back end, front
  end, logger, ...) are running in the same process.
- [MultiProcessRobotData](@ref robot_interfaces::MultiProcessRobotData):  Uses
  shared memory for inter-process communication.  Use this if back end and front
  end are running in separate processes.


See the
[demos](https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos)
for implementations with both the single and the multi process RobotData.
