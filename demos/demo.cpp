/**
 * @file demo.cpp
 * @author Vincent Berenz
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 *
 * @brief Minimal demo of robot driver, backend and frontend
 */

#include "robot_interfaces/example.hpp"
#include "robot_interfaces/monitored_robot_driver.hpp"
#include "robot_interfaces/robot.hpp"
#include "robot_interfaces/robot_backend.hpp"
#include "robot_interfaces/robot_driver.hpp"
#include "robot_interfaces/robot_frontend.hpp"
#include "robot_interfaces/status.hpp"

#include <memory>

/**
 * \example demo.cpp
 * This demo shows robot_interfaces of a
 * dummy "2dof" robot, in which a dof "position"
 * is represented by an integer
 */

using namespace robot_interfaces::example;

int main()
{
    typedef robot_interfaces::RobotBackend<Action, Observation> Backend;
    typedef robot_interfaces::SingleProcessRobotData<Action, Observation> Data;
    typedef robot_interfaces::RobotFrontend<Action, Observation> Frontend;

    // max time allowed for the robot to apply an action.
    double max_action_duration_s = 0.02;

    // max time between for 2 successive actions
    double max_inter_action_duration_s = 0.05;

    // demo showing the separated usage of backend and frontend
    {
        std::cout << "\n -- * -- Frontend and Backend -- * --\n" << std::endl;

        std::shared_ptr<Driver> driver_ptr = std::make_shared<Driver>(0, 1000);
        // Wrap the driver in a MonitoredRobotDriver to automatically run a
        // timing watchdog.  If timing is violated, the robot will immediately
        // be shut down.
        // If no time monitoring is needed in your application, you can simply
        // use the `driver_ptr` directly, without the wrapper.
        auto monitored_driver_ptr =
            std::make_shared<robot_interfaces::MonitoredRobotDriver<Driver>>(
                driver_ptr, max_action_duration_s, max_inter_action_duration_s);

        std::shared_ptr<Data> data_ptr = std::make_shared<Data>();

        Backend backend(monitored_driver_ptr, data_ptr);
        backend.initialize();

        Frontend frontend(data_ptr);

        Action action;
        Observation observation;

        // simulated action :
        // 1 dof going from 200 to 300
        // The other going from 300 to 200

        for (uint value = 200; value <= 300; value++)
        {
            action.values[0] = value;
            action.values[1] = 500 - value;
            // this action will be stored at index
            robot_interfaces::TimeIndex index =
                frontend.append_desired_action(action);
            // getting the observation corresponding to the applied
            // action, i.e. at the same index
            observation = frontend.get_observation(index);
            std::cout << "value: " << value << " | ";
            action.print(false);
            observation.print(true);
        }
    }

    // demo representing usage of frontend and backend
    // encapsulated in the same instance
    {
        std::cout << "\n -- * -- Robot -- * --\n" << std::endl;

        typedef robot_interfaces::Robot<Action, Observation, Driver> Robot;

        int min = 0;
        int max = 100;
        Robot robot(
            max_action_duration_s, max_inter_action_duration_s, min, max);

        robot.initialize();

        Action action;
        Observation observation;

        // simulated action :
        // 1 dof going from 200 to 300
        // The other going from 300 to 200

        for (uint value = 200; value <= 300; value++)
        {
            action.values[0] = value;
            action.values[1] = 500 - value;
            // this action will be stored at index
            robot_interfaces::TimeIndex index =
                robot.append_desired_action(action);
            // getting the observation corresponding to the applied
            // action, i.e. at the same index
            observation = robot.get_observation(index);
            std::cout << "value: " << value << " | ";
            action.print(false);
            observation.print(true);
        }
    }
}
