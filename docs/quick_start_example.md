Quick Start Example
===================

Sending actions to and getting observations from the robot is very easy.  See
the following example that simply sends a constant position command:


## Python

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.py}
import robot_interfaces

# Create frontend instance for the TriFinger robot.
# Do not change the arguments here!
robot_data = robot_interfaces.trifinger.MultiProcessData(
    "trifinger", False)
frontend = robot_interfaces.trifinger.Frontend(robot_data)

position = [
     0.0,  # Finger 1, Upper Joint
    -0.9,  # Finger 1, Middle Joint
    -1.7,  # Finger 1, Lower Joint
     0.0,  # Finger 2, Upper Joint
    -0.9,  # Finger 2, Middle Joint
    -1.7,  # Finger 2, Lower Joint
     0.0,  # Finger 3, Upper Joint
    -0.9,  # Finger 3, Middle Joint
    -1.7,  # Finger 3, Lower Joint
]

while True:
    # construct an action with a position command
    action = robot_interfaces.trifinger.Action(position=position)
    # send the action to the robot (will be applied in time step t)
    t = frontend.append_desired_action(action)
    # wait until time step t and get observation
    observation = frontend.get_observation(t)

    print("Observed Position: {}".format(observation.position))

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



## C++

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
#include <robot_interfaces/finger_types.hpp>

// Some convenience typedefs to make the code below more compact
typedef robot_interfaces::TriFingerTypes::MultiProcessData RobotData;
typedef robot_interfaces::TriFingerTypes::Frontend RobotFrontend;
typedef robot_interfaces::TriFingerTypes::Action Action;

int main()
{
    // Create frontend instance for the TriFinger robot.
    // Do not change the arguments here!
    auto robot_data = std::make_shared<RobotData>("trifinger", false);
    auto frontend = RobotFrontend(robot_data);

    Action::Vector position;  // <- this is an "Eigen::Vector9d"
    position <<  0.0,  // Finger 1, Upper Joint
                -0.9,  // Finger 1, Middle Joint
                -1.7,  // Finger 1, Lower Joint
                 0.0,  // Finger 2, Upper Joint
                -0.9,  // Finger 2, Middle Joint
                -1.7,  // Finger 2, Lower Joint
                 0.0,  // Finger 3, Upper Joint
                -0.9,  // Finger 3, Middle Joint
                -1.7;  // Finger 3, Lower Joint

    while (true)
    {
        // construct an action with a position command
        Action action = Action::Position(position);
        // send the action to the robot (will be applied in time step t)
        auto t = frontend.append_desired_action(action);
        // wait until time step t and get observation
        auto observation = frontend.get_observation(t);

        std::cout << "Observed Position: "
                  << observation.position
                  << std::endl;
    }

    return 0;
}
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using C++ you need to add the package ``robot_interfaces`` as build
dependency to your package.


