# Changelog

## [Unreleased]
### Added
- Add `Ptr` and `ConstPtr` typedefs in classes for more conveniently defining
  shared pointers.
- Function `create_interface_python_bindings()` to create Python bindings for
  the interface classes (but not the robot-specific action and observation
  classes).
- Method `RobotDriver::get_idle_action()` that is expected to return an action
  that is safe to apply while the robot is idle.

### Changed
- The `create_python_bindings()` function (which includes the N-joint
  action/observation) is moved to `pybind_finger.hpp` and renamed to
  `create_blmc_can_robot_python_bindings()`.
- The backend now applies an "idle action" between initialisation and reception
  of the first action by the user.  The idle action is provided by the robot
  driver.  The default implementation simply creates an action using the default
  constructor but for some robots this might not be appropriate; override the
  method with a custom implementation in this case.
  This change is to prevent hardware timeout issues but can also be useful to
  have nicer behaviour of the robot after initialisation (e.g. by holding the
  joints in place).

### Fixed
- pybind11 build error on Ubuntu 22.04


## [1.2.0] - 2022-06-28
### Added
- Measurement (and optional logging) of the RobotBackend frequency.
- Documentation on setup of the real-time Linux kernel.


## [1.1.0] - 2021-06-28

There is no changelog for this or earlier versions.


[Unreleased]: https://github.com/open-dynamic-robot-initiative/robot_interfaces/compare/v1.2.0...HEAD
[1.2.0]: https://github.com/open-dynamic-robot-initiative/robot_interfaces/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/open-dynamic-robot-initiative/robot_interfaces/releases/tag/v1.1.0
