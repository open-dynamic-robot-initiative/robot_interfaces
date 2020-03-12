/**
 * @file
 * @author Vincent Berenz, Felix Widmaier
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019-2020, Max Planck Gesellschaft.
 *
 * @brief Minimal demo of robot driver/backend running in its own process
 */

#include <memory>

#include "robot_interfaces/robot_backend.hpp"
#include "robot_interfaces/robot_driver.hpp"

#include "types.hpp"

using namespace robot_interfaces::demo;

/**
 * \example demo_multiprocess_backend.cpp
 * Robot backend for a dummy "2dof" robot in a multi process setup.
 */

// TODO put driver in separate file so no duplication to demo.cpp?  Discuss
// with Vincent.

// Send command to the robot and read observation from the robot
// The dof positions simply becomes the ones set by the latest action,
// capped between a min and a max value (0 and 1000)
class Driver : public robot_interfaces::RobotDriver<Action, Observation>
{
public:
    Driver()
    {
    }

    // at init dof are at min value
    void initialize()
    {
        state_[0] = Driver::MIN;
        state_[1] = Driver::MIN;
    }

    // just clip desired values
    // between 0 and 1000
    Action apply_action(const Action &action_to_apply)
    {
        std::cout << "received action ";
        action_to_apply.print(true);

        Action applied;
        for (unsigned int i = 0; i < 2; i++)
        {
            if (action_to_apply.values[i] > Driver::MAX)
            {
                applied.values[i] = Driver::MAX;
            }
            else if (action_to_apply.values[i] < Driver::MIN)
            {
                applied.values[i] = Driver::MIN;
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
        // nothing to do
    }

private:
    int state_[2];

    const static int MAX = 1000;
    const static int MIN = 0;
};

int main()
{
    typedef robot_interfaces::RobotBackend<Action, Observation> Backend;
    typedef robot_interfaces::MultiProcessRobotData<Action, Observation>
        MultiProcessData;

    auto driver_ptr = std::make_shared<Driver>();
    // the backend process acts as master for the shared memory
    auto data_ptr =
        std::make_shared<MultiProcessData>("multiprocess_demo", true);

    // max time allowed for the robot to apply an action.
    double max_action_duration_s = 0.02;

    // max time allowed for 2 successive actions
    double max_inter_action_duration_s = 0.05;

    Backend backend(driver_ptr,
                    data_ptr,
                    max_action_duration_s,
                    max_inter_action_duration_s);
    backend.initialize();

    // TODO would be nicer to check if backend loop is still running
    while (true)
    {
    }
}

