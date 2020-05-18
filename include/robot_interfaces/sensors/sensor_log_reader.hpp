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
    //! @brief Data from the log file.
    std::vector<Observation> data;

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
        cereal::BinaryInputArchive archive(infile);

        archive(data);
    }
};

}  // namespace robot_interfaces
