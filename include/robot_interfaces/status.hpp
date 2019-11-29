#pragma once

#include <string>
#include <vector>
#include <robot_interfaces/loggable.hpp>

namespace robot_interfaces
{

  struct Status : public Loggable
  {
    
    uint32_t action_repetitions = 0;
    
    std::vector<std::string> get_name() override
    {
      return {"Action_repetitions"};
    }
    
    std::vector<std::vector<double>> get_data() override
    {
      return {{static_cast<double>(action_repetitions)}};
    }
    
  };

}
