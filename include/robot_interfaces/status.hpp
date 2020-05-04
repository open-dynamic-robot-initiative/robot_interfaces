///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

/**
 * @file
 * @brief Defines the Status struct.
 */

#pragma once

#include <cereal/types/string.hpp>

#include <robot_interfaces/loggable.hpp>
#include <string>
#include <vector>

namespace robot_interfaces
{
/**
 * @brief Status information from the backend.
 *
 * This struct is used to report status information that is not directly
 * robot-related from the backend to the frontend.
 */
struct Status : public Loggable
{
    enum class ErrorStatus
    {
        NO_ERROR = 0,
        DRIVER_ERROR,
        BACKEND_ERROR
    };

    /**
     * @brief Number of times the current action has been repeated because no
     * new action has been provided.
     */
    uint32_t action_repetitions = 0;

    //! @brief Indicates if there is an error and, if yes, in which component.
    ErrorStatus error_status = ErrorStatus::NO_ERROR;

    /**
     * @brief Message describing the error.
     *
     * Value is undefined if `error_status == NO_ERROR`.
     */
    std::string error_message;

    /**
     * @brief Set error.
     *
     * If another error was set before, the old one is kept and the new one
     * ignored.
     *
     * @param error_type  The type of the error.
     * @param message  Error message.
     */
    void set_error(ErrorStatus error_type, const std::string& message)
    {
        // do not overwrite existing errors
        if (!has_error())
        {
            this->error_status = error_type;
            this->error_message = message;
        }
    }

    /**
     * @brief Check if an error is set.
     *
     * See error_status and error_message for more details on the error.
     */
    bool has_error() const
    {
        return this->error_status != Status::ErrorStatus::NO_ERROR;
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(action_repetitions, error_status, error_message);
    }

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
