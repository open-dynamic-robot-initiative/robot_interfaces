///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Eigen>

#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_frontend.hpp>

/**
* @brief Contains definitions of the methods to be implemented by the n joint
* robot data types. The idea is that any external block such as the logger can
* extract the information about the fields of the Observation and Action structures
* using these methods. 
* **/

namespace robot_interfaces
{
class Loggable
{
public:

  virtual std::vector<std::string> get_name() = 0;
  virtual std::vector<std::vector<double>> get_data() = 0;
};

} //namespace robot_interfaces
