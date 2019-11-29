/**
 * @file demo.cpp
 * @author Vincent Berenz
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 * 
 * @brief Minimal demo of robot driver, backend and frontend
 */

#include "robot_interfaces/robot_driver.hpp"
#include "robot_interfaces/robot_backend.hpp"
#include "robot_interfaces/robot_frontend.hpp"

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
    std::cout << "action: "
	      << values[0] << " "
	      << values[1] << " ";
    if(backline)
      std::cout << "\n";
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
    std::cout << "observation: "
	      << values[0] << " "
	      << values[1] << " ";
    if(backline)
      std::cout << "\n";
  }
  
};


// Send command to the robot and read observation from the robot
// The dof positions simply becomes the ones set by the latest action,
// capped between a min and a max value (0 and 1000)
class Driver : public robot_interfaces::RobotDriver< Action,Observation>
{
  
public:

  Driver(){}

  // at init dof are at min value
  void initialize()
  {
    values_[0] = Driver::MIN;
    values_[1] = Driver::MIN;
  }

  // just clip desired values
  // between 0 and 1000
  Action apply_action(const Action &desired_action)
  {
    Action applied;
    for(unsigned int i=0;i<2;i++)
      {
	if(desired_action.values[i]>Driver::MAX)
	  {
	    applied.values[i]=Driver::MAX;
	  }
	else if(desired_action.values[i]<Driver::MIN)
	  {
	    applied.values[i]=Driver::MIN;
	  }
	else
	  {
	    applied.values[i]=desired_action.values[i];
	  }
	values_[i] = applied.values[i];
      }
    return applied;
  }

  Observation get_latest_observation()
  {
    Observation observation;
    observation.values[0] = values_[0];
    observation.values[1] = values_[1];
    return observation;
  }
    
  void shutdown(){}

private:

  int values_[2];
    
  const static int MAX = 1000;
  const static int MIN = 0;

};



int main()
{

  typedef robot_interfaces::RobotBackend<Action,Observation> Backend;
  typedef Backend::Status Status;
  typedef robot_interfaces::RobotData<Action,Observation,Status> Data;
  typedef robot_interfaces::RobotFrontend<Action,Observation> Frontend;
  
  std::shared_ptr<Driver> driver_ptr(new Driver);
  std::shared_ptr<Data> data_ptr(new Data);

  double max_action_duration_s = 1.0;
  double max_inter_action_duration_s = 1.0;

  Backend backend(driver_ptr,
		  data_ptr,
		  max_action_duration_s,
		  max_inter_action_duration_s);
  backend.initialize();
  
  Frontend frontend(data_ptr);

  Action action;
  Observation observation;

  robot_interfaces::TimeIndex index = frontend.get_current_timeindex();
  
  for(uint value=200;value<=0300;value++)
    {
      action.values[0]=value;
      action.values[1]=500-value;
      frontend.append_desired_action(action);
      frontend.wait_until_timeindex(index+1);
      observation = frontend.get_observation(index+1);
      index++;
      std::cout << "value: " << value << " | ";
      action.print(false);
      observation.print(true);
    }
  
}
