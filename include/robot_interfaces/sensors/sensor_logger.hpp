///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <atomic>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>

#include <cereal/archives/binary.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/vector.hpp>

#include <serialization_utils/gzip_iostream.hpp>

#include "sensor_data.hpp"

namespace robot_interfaces
{
/**
 * @brief Record sensor observations and store them to a file.
 *
 * Fetches observations from the given SensorData and buffers them in memory.
 * Buffered observations can be written to a file.  For writing to file cereal
 * is used, so the Observation type has to be serializable by cereal.
 *
 * Usage Example:
 *
 * @code
 *   auto logger = SensorLogger<int>(sensor_data, BUFFER_LIMIT);
 *   logger.start();
 *   // do something
 *   logger.stop_and_save("/tmp/sensordata.log");
 * @endcode
 *
 *
 * @tparam Observation Typ of the observation that is recorded.
 */
template <typename Observation>
class SensorLogger
{
public:
    typedef std::shared_ptr<SensorData<Observation>> DataPtr;
    typedef typename std::tuple<double, Observation> StampedObservation;

    /**
     * @brief Initialize the logger.
     *
     * @param sensor_data  Pointer to the SensorData instance from which
     *     observations are obtained.
     * @param buffer_limit  Maximum number of observations that are logged.
     *     When this limit is reached, the logger will stop automatically, that
     *     is new observations are not logged anymore.
     */
    SensorLogger(DataPtr sensor_data, size_t buffer_limit)
        : sensor_data_(sensor_data),
          buffer_limit_(buffer_limit),
          enabled_(false)
    {
        // directly reserve the memory for the full buffer so it does not need
        // to move data around during run time
        buffer_.reserve(buffer_limit);
    }

    // reinstate the implicit move constructor
    // See https://stackoverflow.com/a/27474070
    SensorLogger(SensorLogger &&) = default;

    ~SensorLogger()
    {
        stop();
    }

    /**
     * @brief Start logging.
     *
     * If the logger is already running, this is a noop.
     */
    void start()
    {
        if (!enabled_)
        {
            enabled_ = true;
            buffer_thread_ =
                std::thread(&SensorLogger<Observation>::loop, this);
        }
    }

    /**
     * @brief Stop logging.
     *
     * If the logger is already stopped, this is a noop.
     */
    void stop()
    {
        enabled_ = false;
        if (buffer_thread_.joinable())
        {
            buffer_thread_.join();
        }
    }

    //! @brief Clear the log buffer.
    void reset()
    {
        buffer_.clear();
    }

    /**
     * @brief Stop logging and save logged messages to a file.
     *
     * @param filename Path to the output file.  Existing files will be
     *     overwritten.
     */
    void stop_and_save(const std::string &filename)
    {
        stop();

        std::ofstream outfile(filename, std::ios::binary);

        auto outfile_compressed = serialization_utils::gzip_ostream(outfile);
        cereal::BinaryOutputArchive archive(*outfile_compressed);

        // add version information to the output file (this can be used while
        // loading when the data format changes
        const std::uint32_t format_version = 1;

        archive(format_version, buffer_);
    }

private:
    DataPtr sensor_data_;
    std::vector<StampedObservation> buffer_;
    size_t buffer_limit_;
    std::thread buffer_thread_;
    bool enabled_;

    //! Get observations from sensor_data_ and add them to the buffer.
    void loop()
    {
        auto t = sensor_data_->observation->oldest_timeindex();

        while (enabled_)
        {
            // wait for the next time step but check if the logger was stopped
            // from time to time
            constexpr double wait_timeout_s = 0.2;
            while (!sensor_data_->observation->wait_for_timeindex(
                t, wait_timeout_s))
            {
                if (!enabled_)
                {
                    return;
                }
            }

            try
            {
                // FIXME: this will block if the sensor has stopped
                auto timestamp = sensor_data_->observation->timestamp_ms(t);
                auto observation = (*sensor_data_->observation)[t];
                buffer_.push_back(std::make_tuple(timestamp, observation));
            }
            catch (const std::invalid_argument &e)
            {
                std::cerr << "ERROR: " << e.what() << "\nSkip observation."
                          << std::endl;
            }
            t++;

            // Stop logging if buffer limit is reached
            if (buffer_.size() >= buffer_limit_)
            {
                std::cerr << "WARNING: SensorLogger buffer limit is reached.  "
                             "Stop logging."
                          << std::endl;
                enabled_ = false;
            }
        }
    }
};

}  // namespace robot_interfaces
