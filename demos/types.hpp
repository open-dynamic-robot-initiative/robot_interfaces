/**
 * @file
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019-2020, Max Planck Gesellschaft.
 *
 * @brief Simple Action and Observation types that are used by some demos.
 */

#pragma once

namespace robot_interfaces
{
namespace demo
{
/**
 * Actions to be performed by robot, will be received by Driver.  An action
 * simply encapsulate two desired position value, one for each dof
 */
class Action
{
public:
    int values[2];

    void print(bool backline) const
    {
        std::cout << "action: " << values[0] << " " << values[1] << " ";
        if (backline) std::cout << "\n";
    }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(values);
    }
};

/**
 * Read from the robot by Driver. An observation is the current position for
 * each dof.
 */
class Observation
{
public:
    int values[2];

    void print(bool backline) const
    {
        std::cout << "observation: " << values[0] << " " << values[1] << " ";
        if (backline) std::cout << "\n";
    }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(values);
    }
};

}  // namespace demo
}  // namespace robot_interfaces
