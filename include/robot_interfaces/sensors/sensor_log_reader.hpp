/**
 * @file
 * @brief API to read the data from a sensor log file.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <fstream>
#include <vector>

#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

namespace robot_interfaces
{
/**
 * @brief Read the data from a sensor log file.
 *
 * The data is read from the specified file and stored to the `data` member
 * where it can be accessed.
 *
 * @tparam Observation Type of the sensor observation.
 */
template <typename Observation>
class SensorLogReader
{
public:
    typedef typename std::tuple<double, Observation> StampedObservation;

    //! @brief Observations from the log file.
    //! @todo rename to "observations"
    std::vector<Observation> data;

    //! @brief Timestamps of the time series from which the observations were
    //! logged.
    std::vector<double> timestamps;

    //! @copydoc SensorLogReader::read_file()
    SensorLogReader(const std::string &filename)
    {
        read_file(filename);
    }

    /**
     * @brief Read data from the specified file.
     *
     * The data is stored to SensorLogReader::data.
     *
     * @param filename Path to the sensor log file.
     */
    void read_file(const std::string &filename)
    {
        std::ifstream infile(filename, std::ios::binary);
        if (!infile)
        {
            throw std::runtime_error("Failed to open file " + filename);
        }

        cereal::BinaryInputArchive archive(infile);

        std::uint32_t format_version;
        archive(format_version);

        if (format_version != 1)
        {
            throw std::runtime_error("Incompatible log file format.");
        }

        std::vector<StampedObservation> stamped_data;
        archive(stamped_data);

        data.reserve(stamped_data.size());
        timestamps.reserve(stamped_data.size());
        for (auto [timestamp, observation] : stamped_data)
        {
            data.push_back(observation);
            timestamps.push_back(timestamp);
        }
    }
};

}  // namespace robot_interfaces
