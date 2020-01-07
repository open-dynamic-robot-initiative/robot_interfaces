///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <robot_interfaces/loggable.hpp>
#include <string>
#include <vector>

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

}  // namespace robot_interfaces
