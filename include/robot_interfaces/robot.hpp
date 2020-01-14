///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <robot_interfaces/robot_frontend.hpp>
#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>

namespace robot_interfaces
{

/**
 * @brief RobotFrontend that construct and encapsulates
 * its related RobotBackend
 */
template<typename Action, typename Observation>
class Robot : public RobotFrontend<Action,Observation>
{

  typedef RobotData<Action,Observation,Status> Data;
  typedef std::shared_ptr<Data> DataPtr;

public:
  
  /**
   * @param robot_driver Driver instance for the actual robot. This is
   * internally wrapped in a MonitoredRobotDriver for increased safety.
   * @param max_action_duration_s See MonitoredRobotDriver.
   * @param max_inter_action_duration_s See MonitoredRobotDriver.
   */
  Robot(std::shared_ptr<RobotDriver<Action, Observation>> robot_driver,
	double max_action_duration_s,
	double max_inter_action_duration_s,
	DataPtr data_ptr = std::make_shared<Data>())
    : RobotFrontend<Action,Observation>(data_ptr),
    data_ptr_(data_ptr),
    backend_(robot_driver,
	     data_ptr_,
	     max_action_duration_s,
	     max_inter_action_duration_s)
  {}

  /**
   * initialize the backend
   */
  void initialize()
  {
    backend_.initialize();
  }
        
  /**
   * return the data shared by the frontend and the backend.
   */
  DataPtr get_data()
  {
    return data_ptr_;
  }
        
private:

  DataPtr data_ptr_;
  RobotBackend<Action,Observation> backend_;
        
};
    
}
