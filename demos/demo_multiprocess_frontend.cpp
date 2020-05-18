/**
 * @file
 * @author Vincent Berenz, Felix Widmaier
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019-2020, Max Planck Gesellschaft.
 *
 * @brief Minimal demo of robot frontend running in its own process
 */

#include <memory>

#include "robot_interfaces/robot_frontend.hpp"

#include "types.hpp"

using namespace robot_interfaces::demo;

/**
 * \example demo_multiprocess_frontend.cpp
 * Robot frontend for a dummy "2dof" robot in a multi process setup.
 */

int main()
{
    typedef robot_interfaces::RobotFrontend<Action, Observation> Frontend;
    typedef robot_interfaces::MultiProcessRobotData<Action, Observation>
        MultiProcessData;

    // The shared memory is managed by the backend process, so set the
    // is_master argument to false.
    auto data_ptr =
        std::make_shared<MultiProcessData>("multiprocess_demo", false);
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
