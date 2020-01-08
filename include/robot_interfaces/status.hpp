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
    enum class ErrorStatus
    {
        NO_ERROR = 0,
        DRIVER_ERROR,
        BACKEND_ERROR
    };

    uint32_t action_repetitions = 0;
    ErrorStatus error_status = ErrorStatus::NO_ERROR;
    std::string error_message;

    std::vector<std::string> get_name() override
    {
        return {"action_repetitions", "error_status"};
    }

    std::vector<std::vector<double>> get_data() override
    {
        // FIXME error message cannot be logged because only numeric types are
        // supported
        return {{static_cast<double>(action_repetitions)},
                {static_cast<double>(error_status)}};
    }
};

}  // namespace robot_interfaces
