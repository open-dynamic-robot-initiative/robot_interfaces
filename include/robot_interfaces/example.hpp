/**
 * @file
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 *
 * @brief Example driver and types for demo and testing purposes.
 */

#include <unistd.h>
#include <iostream>
#include <vector>

#include <robot_interfaces/loggable.hpp>
#include <robot_interfaces/robot_driver.hpp>

namespace robot_interfaces
{
namespace example
{
/**
 * @brief Actions to be performed by robot, will be received by Driver.
 *
 * An action simply encapsulate two desired position value, one for each DOF.
 */
class Action : public Loggable
{
public:
    int values[2];

    void print(bool backline)
    {
        std::cout << "action: " << values[0] << " " << values[1] << " ";
        if (backline)
        {
            std::cout << "\n";
        }
    }

    // the following methods are only needed when you want to use RobotLogger

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(values[0], values[1]);
    }

    virtual std::vector<std::string> get_name()
    {
        return {"values"};
    }

    virtual std::vector<std::vector<double>> get_data()
    {
        return {
            {static_cast<double>(values[0]), static_cast<double>(values[1])}};
    }
};

/**
 * @brief Observation read from the robot by Driver
 *
 * An observation is the current position for each DOF.
 */
class Observation : public Loggable
{
public:
    int values[2];

    void print(bool backline)
    {
        std::cout << "observation: " << values[0] << " " << values[1] << " ";
        if (backline)
        {
            std::cout << "\n";
        }
    }

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(values[0], values[1]);
    }

    virtual std::vector<std::string> get_name()
    {
        return {"values"};
    }

    virtual std::vector<std::vector<double>> get_data()
    {
        return {
            {static_cast<double>(values[0]), static_cast<double>(values[1])}};
    }
};

/**
 * @brief Example Robot Driver.
 *
 * Send command to the robot and read observation from the robot.
 * The DOF positions simply becomes the ones set by the latest action, capped
 * between a min and a max value.
 */
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

    std::optional<std::string> get_error()
    {
        return std::nullopt;  // no error
    }

    void shutdown()
    {
    }

private:
    int state_[2];
    int min_;
    int max_;
};

}  // namespace example
}  // namespace robot_interfaces
