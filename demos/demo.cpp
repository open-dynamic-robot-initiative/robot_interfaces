/**
 * @file demo.cpp
 * @author Vincent Berenz
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 *
 * @brief Minimal demo of robot driver, backend and frontend
 */

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

// Actions to be performed by robot, will be received by Driver
// An action simply encapsulate two desired position value,
// one for each dof
class Action
{
public:
    int values[2];

    void print(bool backline)
    {
        std::cout << "action: " << values[0] << " " << values[1] << " ";
        if (backline) std::cout << "\n";
    }
};

// Read from the robot by Driver
// An observation is the current position
// for each dof
class Observation
{
public:
    int values[2];

    void print(bool backline)
    {
        std::cout << "observation: " << values[0] << " " << values[1] << " ";
        if (backline) std::cout << "\n";
    }
};

// Send command to the robot and read observation from the robot
// The dof positions simply becomes the ones set by the latest action,
// capped between a min and a max value (0 and 1000)
class Driver : public robot_interfaces::RobotDriver<Action, Observation>
{
public:
    Driver(int min, int max) : min_(min), max_(max)
    {
    }

    // at init dof are at min value
    void initialize()
    {
        state_[0] = min_;
        state_[1] = min_;
    }

    // just clip desired values
    // between 0 and 1000
    Action apply_action(const Action &action_to_apply)
    {
        Action applied;
        for (unsigned int i = 0; i < 2; i++)
        {
            if (action_to_apply.values[i] > max_)
            {
                applied.values[i] = max_;
            }
            else if (action_to_apply.values[i] < min_)
            {
                applied.values[i] = min_;
            }
            else
            {
                applied.values[i] = action_to_apply.values[i];
            }
            // simulating the time if could take for a real
            // robot to perform the action
            usleep(1000);
            state_[i] = applied.values[i];
        }
        return applied;
    }

    Observation get_latest_observation()
    {
        Observation observation;
        observation.values[0] = state_[0];
        observation.values[1] = state_[1];
        return observation;
    }

    std::string get_error()
    {
        return "";  // no error
    }

    void shutdown()
    {
    }

private:
    int state_[2];
    int min_;
    int max_;
};

int main()
{
    typedef robot_interfaces::RobotBackend<Action, Observation> Backend;
    typedef robot_interfaces::RobotData<Action,
                                        Observation,
                                        robot_interfaces::Status>
        Data;
    typedef robot_interfaces::RobotFrontend<Action, Observation> Frontend;

    // max time allowed for the robot to apply an action.
    double max_action_duration_s = 0.02;

    // max time allowed for 2 successive actions
    double max_inter_action_duration_s = 0.05;

    // demo showing the separated usage of backend and frontend
    {
        std::cout << "\n -- * -- Frontend and Backend -- * --\n" << std::endl;

        std::shared_ptr<Driver> driver_ptr = std::make_shared<Driver>(0, 1000);
        std::shared_ptr<Data> data_ptr = std::make_shared<Data>();

        Backend backend(driver_ptr,
                        data_ptr,
                        max_action_duration_s,
                        max_inter_action_duration_s);
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
