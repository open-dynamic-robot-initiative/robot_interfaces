/**
 * @file
 * @brief Convert old TriFinger robot logs from before the fix in
 *        Status::error_message.
 *
 * In the past Status::error_message was a variable-sized std::string.  Since
 * this is incompatible with the fixed-size requirement for shared memory time
 * series, it was changed to a fixed-size char-array in commit 4ca02a17.
 * Unfortunately, this makes old logfiles incompatible with the RobotLogReader
 * using the fixed type.
 *
 * This file provides a utility to load log files of the old format and convert
 * them to the new format, so that they can be processed with the latest version
 * of the code.
 *
 * @copyright Copyright (c) 2021, Max Planck Gesellschaft.
 * @license BSD 3-clause
 */
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <cereal/types/string.hpp>

#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/status.hpp>

using namespace robot_interfaces;

//! Old version of the Status message (taken from commit 4ca02a17^, stripped
//! down to the relevant parts).
struct OldStatus
{
    uint32_t action_repetitions = 0;
    Status::ErrorStatus error_status = Status::ErrorStatus::NO_ERROR;
    std::string error_message;

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(action_repetitions, error_status, error_message);
    }
};

typedef RobotBinaryLogReader<TriFingerTypes::Action,
                             TriFingerTypes::Observation,
                             OldStatus>
    OldLogReader;

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Error: Invalid number of arguments." << std::endl;
        std::cerr << "Usage: " << argv[0] << " <old_logfile> <new_logfile>"
                  << std::endl;
        return 1;
    }

    std::string old_logfile(argv[1]);
    std::string new_logfile(argv[2]);

    if (!std::filesystem::is_regular_file(old_logfile))
    {
        std::cout << "Error: Input " << old_logfile << " is not a regular file."
                  << std::endl;
        return 2;
    }
    if (std::filesystem::exists(new_logfile))
    {
        std::cout << "Error: Output destination " << new_logfile
                  << " already exists." << std::endl;
        return 3;
    }

    OldLogReader old_log(old_logfile);
    TriFingerTypes::BinaryLogReader new_log;

    // copy data
    for (OldLogReader::LogEntry old_entry : old_log.data)
    {
        TriFingerTypes::BinaryLogReader::LogEntry new_entry;

        // copy all fields that are unchanged
        new_entry.timeindex = old_entry.timeindex;
        new_entry.timestamp = old_entry.timestamp;
        new_entry.observation = old_entry.observation;
        new_entry.desired_action = old_entry.desired_action;
        new_entry.applied_action = old_entry.applied_action;

        // copy status
        new_entry.status.action_repetitions =
            old_entry.status.action_repetitions;
        new_entry.status.set_error(old_entry.status.error_status,
                                   old_entry.status.error_message);

        new_log.data.push_back(new_entry);
    }

    new_log.write_file(new_logfile);
}
