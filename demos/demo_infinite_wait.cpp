#include "robot_interfaces/robot.hpp"
#include "robot_interfaces/robot_backend.hpp"
#include "robot_interfaces/robot_driver.hpp"
#include "robot_interfaces/robot_frontend.hpp"
#include "robot_interfaces/status.hpp"


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
    typedef robot_interfaces::SingleProcessRobotData<Action,
                                        Observation>
        Data;
    typedef robot_interfaces::RobotFrontend<Action, Observation> Frontend;

    // max time allowed for the robot to apply an action.
    double max_action_duration_s = std::numeric_limits<double>::infinity();

    // max time allowed for 2 successive actions
    double max_inter_action_duration_s = std::numeric_limits<double>::infinity();

    std::shared_ptr<Driver> driver_ptr = std::make_shared<Driver>(0, 1000);
    std::shared_ptr<Data> data_ptr = std::make_shared<Data>();

    Backend backend(driver_ptr,
		    data_ptr,
		    max_action_duration_s,
		    max_inter_action_duration_s,
            false);
    backend.initialize();
    
    Frontend frontend(data_ptr);
    
    Action action;

    action.values[0] = 0;
    action.values[1] = 0;
    frontend.append_desired_action(action);

    usleep(2e6);

}
