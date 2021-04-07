/**
 * @file
 * @brief API to read the data from a robot log file.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <fstream>
#include <vector>

#include <cereal/archives/binary.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/vector.hpp>

#include <serialization_utils/gzip_iostream.hpp>

#include <robot_interfaces/robot_log_entry.hpp>
#include <robot_interfaces/status.hpp>

namespace robot_interfaces
{
/**
 * @brief Read the data from a robot log file.
 *
 * The data is read from the specified file and stored to the `data` member
 * where it can be accessed.
 */
template <typename Action, typename Observation, typename Status_t = Status>
class RobotBinaryLogReader
{
public:
    static constexpr uint32_t FORMAT_VERSION = 2;

    typedef RobotLogEntry<Action, Observation, Status_t> LogEntry;

    std::vector<LogEntry> data;

    RobotBinaryLogReader()
    {
    }

    //! @copydoc RobotBinaryLogReader::read_file()
    RobotBinaryLogReader(const std::string &filename)
    {
        read_file(filename);
    }

    /**
     * @brief Read data from the specified file.
     *
     * The data is stored to RobotBinaryLogReader::data.
     *
     * @param filename Path to the robot log file.
     */
    void read_file(const std::string &filename)
    {
        std::ifstream infile(filename, std::ios::binary);
        if (!infile)
        {
            throw std::runtime_error("Failed to open file " + filename);
        }

        auto infile_compressed = serialization_utils::gzip_istream(infile);
        cereal::BinaryInputArchive archive(*infile_compressed);

        std::uint32_t format_version;
        archive(format_version);

        if (format_version != FORMAT_VERSION)
        {
            throw std::runtime_error("Incompatible log file format.");
        }

        archive(data);
    }

    /**
     * @brief Write data to the specified file.
     *
     * @param filename Path to the output file.
     */
    void write_file(const std::string &filename)
    {
        std::ofstream outfile(filename, std::ios::binary);
        if (!outfile)
        {
            throw std::runtime_error("Failed to open file " + filename);
        }
        auto outfile_compressed = gzip_ostream(outfile);
        cereal::BinaryOutputArchive archive(*outfile_compressed);

        archive(FORMAT_VERSION, data);
    }
};

}  // namespace robot_interfaces
