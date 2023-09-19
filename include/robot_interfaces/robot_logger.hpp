///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <thread>

#include <cereal/archives/binary.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/vector.hpp>

#include <serialization_utils/gzip_iostream.hpp>

#include <robot_interfaces/loggable.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_log_entry.hpp>
#include <robot_interfaces/status.hpp>

namespace robot_interfaces
{
/**
 * @brief Log robot data (observations, actions, status) to file.
 *
 * Logs for each time step the time index, timestamp, and the values
 * Observation, Action, and Status.  The data is written to a text file, one
 * line per time step with values separated by spaces.  This format can easily
 * be read e.g. with NumPy or Pandas.
 *
 * There are different ways of using the logger:
 *
 *  1. Using @ref start() and @ref stop_and_save():  The logger runs a thread in
 *     which it copies all data to an internal buffer while the robot is
 *     running.  When calling @ref stop_and_save() the content of the buffer is
 *     stored to a file (either binary or text).
 *     This is the recommended method for most applications.
 *  2. Using @ref save_current_robot_data() or
 *     @ref save_current_robot_data_binary():
 *     Call this to log data that is currently held in the robot data time
 *     series.  For this method, the logger doesn't use an own buffer, so no
 *     data is copied while the robot is running.  However, the possible time
 *     span for logging is limited by the size of the robot data time series.
 *
 * @tparam Action  Type of the robot action.  Must derive from Loggable.
 * @tparam Observation  Type of the robot observation.  Must derive from
 *                      Loggable.
 */
template <typename Action, typename Observation>
class RobotLogger
{
public:
    // Verify that the template types are based on Loggable.
    static_assert(std::is_base_of<Loggable, Action>::value,
                  "Action must derive from Loggable");
    static_assert(std::is_base_of<Loggable, Observation>::value,
                  "Observation must derive from Loggable");
    static_assert(std::is_base_of<Loggable, Status>::value,
                  "Status must derive from Loggable");

    typedef RobotLogEntry<Action, Observation> LogEntry;

    //! @brief Enumeration of possible log file formats.
    enum class Format
    {
        BINARY,
        CSV,
        CSV_GZIP
    };

    /**
     * @brief Initialize logger.
     *
     * @param robot_data  Pointer to the robot data instance.
     * @param block_size  Block size for writing data to the file when running
     *     the logger in the background.
     */
    RobotLogger(
        std::shared_ptr<robot_interfaces::RobotData<Action, Observation>>
            robot_data,
        size_t buffer_limit,
        int block_size = 100)
        : logger_data_(robot_data),
          buffer_limit_(buffer_limit),
          block_size_(block_size),
          stop_was_called_(false),
          enabled_(false)
    {
        // directly reserve the memory for the full buffer so it does not need
        // to move data around during run time
        buffer_.reserve(buffer_limit);
    }

    // reinstate the implicit move constructor
    // See https://stackoverflow.com/a/27474070
    RobotLogger(RobotLogger &&) = default;

    virtual ~RobotLogger()
    {
        stop();
    }

    // ### Methods for logging with the internal buffer

    /**
     * @brief Start logging using an internal buffer.
     *
     * The buffer is limited by the `buffer_limit` argument of the constructor.
     * If the limit is reached, the logger stops automatically.
     *
     * If the logger is already running, this is a noop.
     */
    void start()
    {
        if (!enabled_)
        {
            enabled_ = true;
            thread_ = std::thread(
                &RobotLogger<Action, Observation>::buffer_loop, this);
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
        if (thread_.joinable())
        {
            thread_.join();
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
    void stop_and_save(const std::string &filename, Format log_format)
    {
        stop();

        switch (log_format)
        {
            case Format::BINARY:
                save_buffer_binary(filename);
                break;
            case Format::CSV:
                save_buffer_text(filename, false);
                break;
            case Format::CSV_GZIP:
                save_buffer_text(filename, true);
                break;
        }
    }

    // ### Methods for directly logging the content of the time series

    /**
     * @brief Write current content of robot data to CSV log file.
     *
     * Write data of the time steps `[start_index, end_index)` to a log file.
     * This assumes that the specified range is completely included in the
     * active robot data buffer.  If this is not the case, only the time steps
     * which are still held in the buffer will be logged.
     *
     * With the default values for start_index and end_index, the whole content
     * of the current buffer is logged.
     *
     * @param filename  Path to the log file.  Existing files will be
     *     overwritten!
     * @param start_index  Time index at which to start logging.  If not
     *     specified, the whole buffer is logged.
     * @param end_index  Time index at which to stop logging.  This is
     *     exclusive, i.e. the specified time index itself will not be part of
     *     the log.  If set to a negative value (default) or a value greater
     *     than the newest time index, the newest time index is used instead
     *     (see @ref newest_timeindex()).
     * @throw std::runtime_error If called while the logger thread is running.
     *     In case the logger thread was started via `start()`, it needs to be
     *     stopped by calling `stop()` before `save_current_robot_data()` can be
     *     used.
     */
    void save_current_robot_data(const std::string &filename,
                                 long int start_index = 0,
                                 long int end_index = -1)
    {
        if (enabled_)
        {
            throw std::runtime_error(
                "RobotLogger is currently running.  Call stop() first.");
        }

        if (logger_data_->observation->length() == 0)
        {
            std::cout
                << "Warning: RobotLogger buffer is empty.  Nothing to write."
                << std::endl;
            return;
        }

        write_header_to_file(filename);

        // set end_index to the current time index if not set or if it is in the
        // future.
        long int t = logger_data_->observation->newest_timeindex();
        if (end_index < 0)
        {
            end_index = t;
        }
        else
        {
            end_index = std::min(t, end_index);
        }

        append_robot_data_to_file(
            filename, start_index, end_index - start_index);
    }

    //! @brief Like save_current_robot_data but using binary file format.
    void save_current_robot_data_binary(const std::string &filename,
                                        long int start_index = 0,
                                        long int end_index = -1)
    {
        if (enabled_)
        {
            throw std::runtime_error(
                "RobotLogger is currently running.  Call stop() first.");
        }

        if (logger_data_->observation->length() == 0)
        {
            std::cout
                << "Warning: RobotLogger buffer is empty.  Nothing to write."
                << std::endl;
            return;
        }

        // set end_index to the current time index if not set or if it is in the
        // future.
        long int t = logger_data_->observation->newest_timeindex();
        if (end_index < 0)
        {
            end_index = t;
        }
        else
        {
            end_index = std::min(t, end_index);
        }

        long int block_size = end_index - start_index;

        std::vector<LogEntry> log_data;
        log_data.reserve(block_size);

        for (long int t = start_index;
             t < std::min(start_index + block_size,
                          logger_data_->observation->newest_timeindex());
             t++)
        {
            try
            {
                LogEntry entry;
                entry.timeindex = t;
                entry.applied_action = (*logger_data_->applied_action)[t];
                entry.desired_action = (*logger_data_->desired_action)[t];
                entry.observation = (*logger_data_->observation)[t];
                entry.status = (*logger_data_->status)[t];
                entry.timestamp = logger_data_->observation->timestamp_s(t);

                log_data.push_back(entry);
            }
            catch (const std::invalid_argument &e)
            {
                auto t_oldest = logger_data_->observation->oldest_timeindex();
                auto diff = t_oldest - t;

                std::cout << "Warning: Trying to log time step " << t
                          << " which is not in the buffer anymore.  Skip "
                          << diff << " steps to time step " << (t_oldest + 1)
                          << std::endl;

                // Note that t is incremented in the next iteration, so logging
                // continues from t_oldest + 1.  This means that one potentially
                // available step is dropped but it lowers the risk that the
                // same issue happens immediately again as t_oldest may drop out
                // of the buffer until it is processed in the next iteration.
                t = t_oldest;
            }
        }

        std::ofstream outfile(filename, std::ios::binary);

        auto outfile_compressed = serialization_utils::gzip_ostream(outfile);
        cereal::BinaryOutputArchive archive(*outfile_compressed);

        // add version information to the output file (this can be used while
        // loading when the data format changes
        const std::uint32_t format_version = 2;

        archive(format_version, log_data);
    }

private:
    std::thread thread_;

    std::shared_ptr<robot_interfaces::RobotData<Action, Observation>>
        logger_data_;

    std::vector<LogEntry> buffer_;
    size_t buffer_limit_;

    int block_size_;
    long int index_;

    std::atomic<bool> stop_was_called_;
    std::atomic<bool> enabled_;

    std::string output_file_name_;

    /**
     * @brief To get the title of the log file, describing all the
     * information that will be logged in it.
     *
     * @return header The title of the log file.
     */
    std::vector<std::string> construct_header() const
    {
        Action action;
        Observation observation;
        Status status;

        std::vector<std::string> observation_names = observation.get_name();
        std::vector<std::string> action_names = action.get_name();
        std::vector<std::string> status_name = status.get_name();

        std::vector<std::vector<double>> observation_data =
            observation.get_data();
        std::vector<std::vector<double>> action_data = action.get_data();
        std::vector<std::vector<double>> status_data = status.get_data();

        std::vector<std::string> header;

        header.push_back("#time_index");
        header.push_back("timestamp");

        append_names_to_header("status", status_name, status_data, header);
        append_names_to_header(
            "observation", observation_names, observation_data, header);
        append_names_to_header(
            "applied_action", action_names, action_data, header);
        append_names_to_header(
            "desired_action", action_names, action_data, header);

        return header;
    }

    /**
     * @brief Fills in the name information of each field to be
     * logged according to the size of the field.
     *
     * @param identifier The structure the field corresponds to
     * @param field_name The name of the field
     * @param field_data The field data
     * @param &header Reference to the header of the log file
     */
    void append_names_to_header(
        const std::string &identifier,
        const std::vector<std::string> &field_name,
        const std::vector<std::vector<double>> &field_data,
        std::vector<std::string> &header) const
    {
        for (size_t i = 0; i < field_name.size(); i++)
        {
            if (field_data[i].size() == 1)
            {
                std::string temp = identifier + "_" + field_name[i];
                header.push_back(temp);
            }
            else
            {
                for (size_t j = 0; j < field_data[i].size(); j++)
                {
                    std::string temp = identifier + "_" + field_name[i] + "_" +
                                       std::to_string(j);
                    header.push_back(temp);
                }
            }
        }
    }

    /**
     * @brief Write CSV header to the log file.  This overwrites existing files!
     */
    void write_header_to_file(const std::string &filename) const
    {
        std::ofstream output_file(filename);
        write_header_to_stream(output_file);
        output_file.close();
    }

    /**
     * @brief Write CSV header to the output stream.
     */
    void write_header_to_stream(std::ostream &output) const
    {
        std::ostream_iterator<std::string> string_iterator(output, " ");

        std::vector<std::string> header = construct_header();

        std::copy(header.begin(), header.end(), string_iterator);
        output << std::endl;
    }

    /**
     * @brief Writes a block of time steps to the log file.
     *
     * @param start_index  Time index marking the beginning of the block.
     * @param block_size  Number of time steps that are written to the log file.
     */
    void append_robot_data_to_file(const std::string &filename,
                                   long int start_index,
                                   long int block_size)
    {
        std::ofstream output_file;
        output_file.open(filename, std::ios_base::app);
        output_file.precision(27);

        for (long int t = start_index;
             t < std::min(start_index + block_size,
                          logger_data_->observation->newest_timeindex());
             t++)
        {
            try
            {
                Action applied_action = (*logger_data_->applied_action)[t];
                Action desired_action = (*logger_data_->desired_action)[t];
                Observation observation = (*logger_data_->observation)[t];
                Status status = (*logger_data_->status)[t];
                auto timestamp = logger_data_->observation->timestamp_s(t);

                output_file << t << " " << timestamp << " ";
                append_field_data_to_file(status.get_data(), output_file);
                append_field_data_to_file(observation.get_data(), output_file);
                append_field_data_to_file(applied_action.get_data(),
                                          output_file);
                append_field_data_to_file(desired_action.get_data(),
                                          output_file);
                output_file << std::endl;
            }
            catch (const std::invalid_argument &e)
            {
                auto t_oldest = logger_data_->observation->oldest_timeindex();
                auto diff = t_oldest - t;

                std::cout << "Warning: Trying to log time step " << t
                          << " which is not in the buffer anymore.  Skip "
                          << diff << " steps to time step " << (t_oldest + 1)
                          << std::endl;

                // Note that t is incremented in the next iteration, so logging
                // continues from t_oldest + 1.  This means that one potentially
                // available step is dropped but it lowers the risk that the
                // same issue happens immediately again as t_oldest may drop out
                // of the buffer until it is processed in the next iteration.
                t = t_oldest;
            }
        }

        output_file.close();
    }

    /**
     * @brief Appends the logger buffer as plain text (CSV) to an output stream.
     *
     * @param output_stream  Output stream to which the log is written.
     */
    void append_logger_buffer(std::ostream &output_stream) const
    {
        // output_file.open(filename, std::ios_base::app);
        output_stream.precision(27);

        for (LogEntry entry : buffer_)
        {
            output_stream << entry.timeindex << " " << entry.timestamp << " ";
            append_field_data_to_file(entry.status.get_data(), output_stream);
            append_field_data_to_file(entry.observation.get_data(),
                                      output_stream);
            append_field_data_to_file(entry.applied_action.get_data(),
                                      output_stream);
            append_field_data_to_file(entry.desired_action.get_data(),
                                      output_stream);
            output_stream << std::endl;
        }
    }

    /**
     * @brief Appends the data corresponding to
     * every field at the same time index to the log file.
     *
     * @param field_data The field data
     */
    void append_field_data_to_file(
        const std::vector<std::vector<double>> &field_data,
        std::ostream &output_stream) const
    {
        std::ostream_iterator<double> double_iterator(output_stream, " ");

        for (auto data : field_data)
        {
            std::copy(data.begin(), data.end(), double_iterator);
        }
    }

    /**
     * @brief Save content of the internal buffer to a binary file.
     *
     * @param filename  Path/name of the output file.
     */
    void save_buffer_binary(const std::string &filename) const
    {
        std::ofstream outfile(filename, std::ios::binary);

        auto outfile_compressed = serialization_utils::gzip_ostream(outfile);
        cereal::BinaryOutputArchive archive(*outfile_compressed);

        // add version information to the output file (this can be used while
        // loading when the data format changes
        const std::uint32_t format_version = 2;

        archive(format_version, buffer_);
    }

    /**
     * @brief Save content of the internal buffer to a CSV file.
     *
     * @param filename  Path/name of the output file
     * @param use_gzip  If true, the output file is gzip-compressed.
     */
    void save_buffer_text(const std::string &filename,
                          bool use_gzip = false) const
    {
        std::ofstream output_file(filename);
        auto output = serialization_utils::gzip_ostream(output_file, use_gzip);

        write_header_to_stream(*output);
        append_logger_buffer(*output);
    }

    /**
     * @brief Writes everything to the log file.
     *
     * It dumps all the data corresponding to block_size_ number of time indices
     * at one go.
     */
    void loop()
    {
        enabled_ = true;

        write_header_to_file(output_file_name_);

        while (!stop_was_called_ &&
               !(logger_data_->desired_action->length() > 0))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        index_ = logger_data_->observation->newest_timeindex();

        while (!stop_was_called_)
        {
            if (index_ + block_size_ <=
                logger_data_->observation->newest_timeindex())
            {
#ifdef VERBOSE
                auto t1 = std::chrono::high_resolution_clock::now();
#endif

                append_robot_data_to_file(
                    output_file_name_, index_, block_size_);

                index_ += block_size_;

#ifdef VERBOSE
                auto t2 = std::chrono::high_resolution_clock::now();
                auto duration =
                    std::chrono::duration_cast<std::chrono::microseconds>(t2 -
                                                                          t1)
                        .count();

                // to print the time taken for one block of data to be logged.
                std::cout << "Time taken for one block of data to be logged: "
                          << duration << std::endl;
#endif
            }
        }

        enabled_ = false;
    }

    //! Get observations from logger_data_ and add them to the buffer.
    void buffer_loop()
    {
        // Get the oldest available timeindex as starting point of the log.  In
        // case there is no data yet (t == EMPTY), start with t = 0.
        auto t = logger_data_->observation->oldest_timeindex(false);
        if (t == time_series::EMPTY)
        {
            t = 0;
        }

        while (enabled_)
        {
            // wait for the next time step but check if the logger was stopped
            // from time to time
            constexpr double wait_timeout_s = 0.2;
            while (!logger_data_->applied_action->wait_for_timeindex(
                t, wait_timeout_s))
            {
                if (!enabled_)
                {
                    return;
                }
            }

            try
            {
                LogEntry entry;

                entry.timeindex = t;
                entry.applied_action = (*logger_data_->applied_action)[t];
                entry.desired_action = (*logger_data_->desired_action)[t];
                entry.observation = (*logger_data_->observation)[t];
                entry.status = (*logger_data_->status)[t];
                entry.timestamp = logger_data_->observation->timestamp_s(t);

                buffer_.push_back(entry);
                t++;
            }
            catch (const std::invalid_argument &e)
            {
                auto t_oldest = logger_data_->observation->oldest_timeindex();
                auto diff = t_oldest - t;

                std::cerr << "ERROR: While logging time step " << t << ": "
                          << e.what() << "\nSkip " << diff << " observation(s)."
                          << std::endl;

                t = t_oldest;
            }

            // Stop logging if buffer limit is reached
            if (buffer_.size() >= buffer_limit_)
            {
                std::cerr << "WARNING: RobotLogger buffer limit is reached.  "
                             "Stop logging."
                          << std::endl;
                enabled_ = false;
            }
        }
    }
};

}  // namespace robot_interfaces
