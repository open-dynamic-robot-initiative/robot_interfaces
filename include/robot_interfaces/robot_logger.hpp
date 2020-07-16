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

#include <Eigen/Eigen>

#include <mpi_cpp_tools/basic_tools.hpp>
#include <mpi_cpp_tools/dynamical_systems.hpp>
#include <mpi_cpp_tools/math.hpp>

#include <robot_interfaces/loggable.hpp>
#include <robot_interfaces/status.hpp>

namespace robot_interfaces
{
/**
 * @brief To log data from *any* robot (real, simulated, fake).
 *
 * The RobotLogger logs the timestamp, the time index, and the values of every
 * Observation, Action, and Status variable. Observation, Action, and Status
 * *must* derive from Loggable. Any further data structure can be logged
 * similarly, which derives from Loggable.
 *
 * @tparam Action
 * @tparam Observation
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

    RobotLogger(
        std::shared_ptr<robot_interfaces::RobotData<Action, Observation>>
            robot_data,
        int block_size = 100)
        : logger_data_(robot_data),
          block_size_(block_size),
          stop_was_called_(false)
    {
    }

    virtual ~RobotLogger()
    {
        stop();
    }

    /**
     * @brief Create the thread for the RobotLogger and start logging.
     *
     * \note
     * Every time you start the logger with the same file name, it will
     * obviously append newer data to the same file. This shouldn't be a
     * problem. But for different log files, specify different file names while
     * starting the logger.
     *
     * @param filename The name of the log file.
     */
    void start(const std::string &filename)
    {
        stop_was_called_ = false;
        output_file_name_ = filename;
        thread_ = std::thread(&RobotLogger<Action, Observation>::loop, this);
    }

    /**
     * @brief Stop logging.
     */
    void stop()
    {
        // FIXME: just calling `stop()` without `start()` first will be bad?!
        stop_was_called_ = true;
        if (thread_.joinable())
        {
            thread_.join();
        }
        append_robot_data_to_file(index_, block_size_);
    }

    void write_current_buffer(const std::string filename, long int start_index)
    {
        write_header_to_file();
        // log from current
        append_robot_data_to_file(start_index,
                                  std::numeric_limits<long int>::max());
    }

private:
    std::thread thread_;

    std::shared_ptr<robot_interfaces::RobotData<Action, Observation>>
        logger_data_;

    int block_size_;
    long int index_;

    std::atomic<bool> stop_was_called_;

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

                std::vector<std::vector<double>> status_data =
                    status.get_data();
                std::vector<std::vector<double>> observation_data =
                    observation.get_data();

                std::vector<std::vector<double>> applied_action_data =
                    applied_action.get_data();

                std::vector<std::vector<double>> desired_action_data =
                    desired_action.get_data();

                output_file_ << t << " "
                             << logger_data_->observation->timestamp_s(t)
                             << " ";

                append_field_data_to_file(status_data);
                append_field_data_to_file(observation_data);
                append_field_data_to_file(applied_action_data);
                append_field_data_to_file(desired_action_data);

                output_file_ << std::endl;
            }

            catch (const std::exception &e)
            {
                std::cout << "Trying to access index older than the "
                             "oldest! Skipping ahead."
                          << std::endl;
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
    }
};

}  // namespace robot_interfaces
