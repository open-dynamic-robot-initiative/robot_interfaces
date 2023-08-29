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
 * Used to report status information that is not directly robot-related from the
 * backend to the frontend.
 */
struct Status : public Loggable
{
    //! Maximum length of error messages (including terminating \0)
    // FIXME this is too short!
    static constexpr unsigned int ERROR_MESSAGE_LENGTH = 64;

    //! @brief Different types of errors that can occur in the backend.
    enum class ErrorStatus
    {
        //! @brief Indicates that there is no error.
        NO_ERROR = 0,

        /**
         * @brief Error reported from the @ref RobotDriver.
         *
         * An error reported by the low level robot driver (see @ref
         * RobotDriver).  This is depending on the driver implementation.  It
         * can, for example, be used to report some hardware failure).
         */
        DRIVER_ERROR,

        /**
         * @brief Error from the @ref RobotBackend.
         *
         * An error which is issued by the back end itself, for example if no
         * new action is provided and the allowed number of repetitions is
         * exceeded.
         */
        BACKEND_ERROR
    };

    /**
     * @brief Number of times the current action has been repeated.
     *
     * If the back end wants to apply the next action but no new action was
     * provided by the user in time, it may (depending on configuration) repeat
     * the previous action.  Each time this happens, `action_repetitions` is
     * increased by one.  Once a new action is provided, it will be reset to
     * zero.
     *
     * See also @ref next-action-not-in-time.
     */
    uint32_t action_repetitions = 0;

    /**
     * @brief Indicates if there is an error and, if yes, in which component.
     *
     * @note If there is an error reported in the status, the robot is not in an
     *       operational state anymore.  Trying to append another action in the
     *       @ref RobotFrontend will result in an exception in this case.
     *
     * @see error_message for more information on the error.
     * @see has_error()
     */
    ErrorStatus error_status = ErrorStatus::NO_ERROR;

    /**
     * @brief Set error.
     *
     * If another error was set before, the old one is kept and the new one
     * ignored.
     *
     * @param error_type  The type of the error.
     * @param message  Error message.  Will be shortened if it exceeds @ref
     *      ERROR_MESSAGE_LENGTH.
     */
    // FIXME don't use std::string for real-time critical code
    void set_error(ErrorStatus error_type, const std::string& message)
    {
        // do not overwrite existing errors
        if (!has_error())
        {
            this->error_status = error_type;

            std::strncpy(
                this->error_message, message.c_str(), ERROR_MESSAGE_LENGTH - 1);
            // in case message is too long and needs to be cut, indicate this by
            // setting ~ as last character
            if (message.size() > ERROR_MESSAGE_LENGTH - 1)
            {
                this->error_message[ERROR_MESSAGE_LENGTH - 2] = '~';
            }
            // make sure it is terminated
            this->error_message[ERROR_MESSAGE_LENGTH - 1] = '\0';
        }
    }

    /**
     * @brief Check if an error is set.
     *
     * @note If there is an error reported in the status, the robot is not in an
     *       operational state anymore.  Trying to append another action in the
     *       @ref RobotFrontend will result in an exception in this case.
     *
     * See @ref error_status and @ref error_message for more details on the
     * error.
     */
    bool has_error() const
    {
        return this->error_status != Status::ErrorStatus::NO_ERROR;
    }

    //! Get the error message as std::string.
    std::string get_error_message() const
    {
        return std::string(this->error_message);
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

private:
    /**
     * @brief Human-readable message describing the error.
     *
     * Value is undefined if `error_status == NO_ERROR`.
     */
    char error_message[ERROR_MESSAGE_LENGTH] = "";
};

}  // namespace robot_interfaces
