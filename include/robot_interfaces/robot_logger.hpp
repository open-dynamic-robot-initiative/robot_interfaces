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
#include <limits>
#include <thread>

#include <robot_interfaces/loggable.hpp>
#include <robot_interfaces/robot_data.hpp>
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
 * There are two different ways of using the logger:
 *
 *  1. Write all the data from the time series to the file in one function call.
 *     Use this if the time series buffer is big enough to cover the whole time
 *     span that you want to log.  This way all the data is written to the file
 *     in the end, when the robot is not moving anymore.  This way it will not
 *     interfere with the running robot but there is the risk of losing all data
 *     in case the software crashes before writing the log.
 *     Use the method `write_current_buffer()` for this.
 *  2. Run the logger in the background and write blocks of data to the log file
 *     while the robot is running.  This has the advantage that arbitrary time
 *     spans can be logged independent of the buffer size of the time series.
 *     Further in case of a software crash not all data will be lost but only
 *     the data since the last block was written.  However, it has the huge
 *     disadvantage that writing to the file may cause delays in the real-time
 *     critical robot code, thus causing the robot to shut down if timing
 *     constraints are violated.
 *     Use the `start()` and `stop()` methods for this.
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
        int block_size = 100)
        : logger_data_(robot_data),
          block_size_(block_size),
          stop_was_called_(false),
          is_running_(false)
    {
    }

    virtual ~RobotLogger()
    {
        stop();
    }

    /**
     * @brief Start a thread to continuously log to file in the background.
     *
     * @see stop()
     * @param filename The name of the log file.  Existing files will be
     *     overwritten!
     */
    void start(const std::string &filename)
    {
        stop_was_called_ = false;
        output_file_name_ = filename;
        thread_ = std::thread(&RobotLogger<Action, Observation>::loop, this);
    }

    /**
     * @brief Stop logging that was started with `start()` previously.
     *
     * Does nothing if logger is not currently running.
     */
    void stop()
    {
        // This is a bit complicated:  In any case, join the thread if it is
        // joinable.  However, only write the remaining data to file if the
        // logging thread is actually running at the moment stop() is called.
        bool still_running = is_running_;

        stop_was_called_ = true;
        if (thread_.joinable())
        {
            thread_.join();
        }

        if (still_running)
        {
            append_robot_data_to_file(index_, block_size_);
        }
    }

    /**
     * @brief Write current content of robot data to log file.
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
     *     stopped by calling `stop()` before `write_current_buffer()` can be
     *     used.
     */
    void write_current_buffer(const std::string filename,
                              long int start_index = 0,
                              long int end_index = -1)
    {
        if (is_running_)
        {
            throw std::runtime_error(
                "RobotLogger is currently running.  Call stop() first.");
        }

        output_file_name_ = filename;
        write_header_to_file();

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

        append_robot_data_to_file(start_index, end_index - start_index);
    }

private:
    std::thread thread_;

    std::shared_ptr<robot_interfaces::RobotData<Action, Observation>>
        logger_data_;

    int block_size_;
    long int index_;

    std::atomic<bool> stop_was_called_;
    std::atomic<bool> is_running_;

    std::ofstream output_file_;
    std::string output_file_name_;

    /**
     * @brief To get the title of the log file, describing all the
     * information that will be logged in it.
     *
     * @return header The title of the log file.
     */
    std::vector<std::string> construct_header()
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
        std::vector<std::string> &header)
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
     * @brief Write the header to the log file.  This overwrites existing files!
     */
    void write_header_to_file()
    {
        output_file_.open(output_file_name_);
        std::ostream_iterator<std::string> string_iterator(output_file_, " ");

        std::vector<std::string> header = construct_header();

        std::copy(header.begin(), header.end(), string_iterator);
        output_file_ << std::endl;

        output_file_.close();
    }

    /**
     * @brief Writes a block of time steps to the log file.
     *
     * @param start_index  Time index marking the beginning of the block.
     * @param block_size  Number of time steps that are written to the log file.
     */
    void append_robot_data_to_file(long int start_index, long int block_size)
    {
        output_file_.open(output_file_name_, std::ios_base::app);
        output_file_.precision(27);

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

                output_file_ << t << " " << timestamp << " ";
                append_field_data_to_file(status.get_data());
                append_field_data_to_file(observation.get_data());
                append_field_data_to_file(applied_action.get_data());
                append_field_data_to_file(desired_action.get_data());
                output_file_ << std::endl;
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

        output_file_.close();
    }

    /**
     * @brief Appends the data corresponding to
     * every field at the same time index to the log file.
     *
     * @param field_data The field data
     */
    void append_field_data_to_file(
        const std::vector<std::vector<double>> &field_data)
    {
        std::ostream_iterator<double> double_iterator(output_file_, " ");

        for (auto data : field_data)
        {
            std::copy(data.begin(), data.end(), double_iterator);
        }
    }

    /**
     * @brief Writes everything to the log file.
     *
     * It dumps all the data corresponding to block_size_ number of time indices
     * at one go.
     */
    void loop()
    {
        is_running_ = true;

        write_header_to_file();

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

                append_robot_data_to_file(index_, block_size_);

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

        is_running_ = false;
    }
};

}  // namespace robot_interfaces
