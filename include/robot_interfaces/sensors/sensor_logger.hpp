///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
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
#include <cereal/types/vector.hpp>

#include "sensor_data.hpp"

namespace robot_interfaces
{
template <typename Observation>
class SensorLogger
{
public:
    typedef std::shared_ptr<SensorData<Observation>> DataPtr;

    SensorLogger(DataPtr sensor_data, size_t buffer_limit)
        : sensor_data_(sensor_data),
          buffer_limit_(buffer_limit),
          enabled_(false)
    {
    }

    // reinstate the implicit move constructor
    // See https://stackoverflow.com/a/27474070
    SensorLogger(SensorLogger &&) = default;

    ~SensorLogger()
    {
        stop();
    }

    void start()
    {
        enabled_ = true;
        buffer_thread_ = std::thread(&SensorLogger<Observation>::loop, this);
    }

    void stop()
    {
        enabled_ = false;
        if (buffer_thread_.joinable())
        {
            buffer_thread_.join();
        }
    }

    void reset()
    {
        buffer_.clear();
    }

    void stop_and_save(const std::string &filename)
    {
        stop();

        std::ofstream outfile(filename, std::ios::binary);
        cereal::BinaryOutputArchive archive(outfile);

        archive(buffer_);
    }

private:
    DataPtr sensor_data_;
    std::vector<Observation> buffer_;
    size_t buffer_limit_;
    std::thread buffer_thread_;
    bool enabled_;

    void loop()
    {
        auto t = sensor_data_->observation->newest_timeindex();

        while (enabled_)
        {
            try
            {
                buffer_.push_back((*sensor_data_->observation)[t]);
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
