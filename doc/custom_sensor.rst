****************
Sensor Interface
****************

``robot_interfaces`` also provides base classes for a pure sensor interface.  In
contrast to a robot, a sensor only provides observations but does not take actions.
Apart from that, the overall structure is mostly the same as for a robot.  That is,
there are :cpp:class:`~robot_interfaces::SensorBackend` and
:cpp:class:`~robot_interfaces::SensorFrontend` which communicate via
:cpp:class:`~robot_interfaces::SensorData` (either single- or multi-process).

For implementing an interface for your sensor, you only need to implement an observation
type and a driver class based on :cpp:class:`~robot_interfaces::SensorDriver`.  Then
simply create data/backend/frontend using those custom types as template arguments.

Optionally, your driver class can also provide a "sensor info" object by implementing
:cpp:func:`~robot_interfaces::SensorDriver::get_sensor_info`.  This object is then
accessible by the user via
:cpp:func:`~robot_interfaces::SensorFrontend::get_sensor_info`.  You may use this, to
provide static information about the sensor, that does not change over time (e.g. frame
rate of a camera).
If you don't implement the corresponding method in the driver, the front end will return
an empty struct as placeholder.
