/**
 * @file
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */
#pragma once

namespace robot_interfaces
{
/**
 * @brief Static class implementing a signal handler and providing information
 *        about SIGINT.
 *
 * Since signal handlers cannot be implemented in a non-static class method, use
 * this globally accessible class as a workaround.  It registers a signal
 * handler provides static methods to poll if certain signals are received.
 */
class GlobalSignalHandler
{
public:
    /**
     * @brief Initialize the signal handler.
     *
     * This must be called before using the class.  Does nothing when called a
     * second time.
     */
    static void initialize();

    //! @brief Handles the signals.  Do not call this directly.
    static void signal_handler(int signal);

    /**
     * @brief Check if a SIGINT was received.
     *
     * @return True if SIGINT was received.
     */
    static bool has_received_sigint();

private:
    GlobalSignalHandler()
    {
        // private constructor to prevent instantiation
    }

    static bool received_sigint_;
};

}  // namespace robot_interfaces
