/**
 * @file
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */
#include <csignal>
#include <robot_interfaces/global_signal_handler.hpp>

namespace robot_interfaces
{
bool GlobalSignalHandler::received_sigint_ = false;

void GlobalSignalHandler::initialize()
{
    static bool is_initialized = false;

    if (!is_initialized)
    {
        std::signal(SIGINT, &GlobalSignalHandler::signal_handler);
        received_sigint_ = false;
        is_initialized = true;
    }
}

void GlobalSignalHandler::signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        received_sigint_ = true;
    }
}

bool GlobalSignalHandler::has_received_sigint()
{
    return received_sigint_;
}

}  // namespace robot_interfaces
