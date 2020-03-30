///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <fstream>
#include <iostream>

#include <chrono>

#include <Eigen/Eigen>

#include <mpi_cpp_tools/basic_tools.hpp>
#include <mpi_cpp_tools/dynamical_systems.hpp>
#include <mpi_cpp_tools/math.hpp>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/timer.hpp>

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
    /**
     * This is to verify that the template types of the RobotLogger are based on
     * Loggable.
     */
    static_assert(std::is_base_of<Loggable, Action>::value,
                  "Action must derive from Loggable");
    static_assert(std::is_base_of<Loggable, Observation>::value,
                  "Observation must derive from Loggable");
    static_assert(std::is_base_of<Loggable, Status>::value,
                  "Status must derive from Loggable");
    /**
     * Currently, the level of generalisability of the logger is that we *know*
     * the following four timeseries structures exist for *any* robot whose data
     * is to be logged- applied_action and desired_action (of type Action),
     * observation (of type Observation), and status (of type Status).
     */
    std::shared_ptr<robot_interfaces::RobotData<Action, Observation>>
        logger_data_;

    int block_size_;
    long int index_;

    bool stop_was_called_;

    std::ofstream output_file_;
    std::string output_file_name_;

    RobotLogger(std::shared_ptr<
                    robot_interfaces::RobotData<Action, Observation>>
                    robot_data,
                int block_size)
        : logger_data_(robot_data),
          block_size_(block_size),
          stop_was_called_(false)
    {
        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
    }

    virtual ~RobotLogger()
    {
        stop();
    }

    /**
     * @brief To get the title of the log file, describing all the
     * information that will be logged in it.
     *
     * @return header The title of the log file.
     */
    std::vector<std::string> get_header()
    {
        Action applied_action;
        Action desired_action;
        Observation observation;
        Status status;

        std::vector<std::string> observation_name = observation.get_name();
        std::vector<std::string> applied_action_name =
            applied_action.get_name();
        std::vector<std::string> desired_action_name =
            desired_action.get_name();
        std::vector<std::string> status_name = status.get_name();

        std::vector<std::vector<double>> observation_data =
            observation.get_data();
        std::vector<std::vector<double>> applied_action_data =
            applied_action.get_data();
        std::vector<std::vector<double>> desired_action_data =
            desired_action.get_data();
        std::vector<std::vector<double>> status_data = status.get_data();

        std::vector<std::string> header;

        header.push_back("#time_index");
        header.push_back("timestamp");

        append_name_to_header("status", status_name, status_data, header);
        append_name_to_header(
            "observation", observation_name, observation_data, header);
        append_name_to_header(
            "applied_action", applied_action_name, applied_action_data, header);
        append_name_to_header(
            "desired_action", desired_action_name, desired_action_data, header);

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
    void append_name_to_header(std::string identifier,
                               std::vector<std::string> field_name,
                               std::vector<std::vector<double>> field_data,
                               std::vector<std::string> &header)
    {
        for (size_t i = 0; i < field_name.size(); i++)
        {
            if (field_data[i].size() == 1)
            {
                std::string temp = field_name[i];
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
     * @brief Writes the header to the log file.
     */
    void append_header_to_file()
    {
        output_file_.open(output_file_name_, std::ios_base::app);
        std::ostream_iterator<std::string> string_iterator(output_file_, " ");

        std::vector<std::string> header = get_header();

        std::copy(header.begin(), header.end(), string_iterator);
        output_file_ << std::endl;

        output_file_.close();
    }

    /**
     * @brief Writes the timestamped robot data at
     * *hopefully* every time index to the log file.
     */
    void append_robot_data_to_file()
    {
        output_file_.open(output_file_name_, std::ios_base::app);
        output_file_.precision(27);

        for (long int j = index_;
             j < std::min(index_ + block_size_,
                          logger_data_->observation->newest_timeindex());
             j++)
        {
            try
            {
                Action applied_action = (*logger_data_->applied_action)[j];
                Action desired_action = (*logger_data_->desired_action)[j];
                Observation observation = (*logger_data_->observation)[j];
                Status status = (*logger_data_->status)[j];

                std::vector<std::vector<double>> status_data =
                    status.get_data();
                std::vector<std::vector<double>> observation_data =
                    observation.get_data();

                std::vector<std::vector<double>> applied_action_data =
                    applied_action.get_data();

                std::vector<std::vector<double>> desired_action_data =
                    desired_action.get_data();

                output_file_ << j << " "
                    << logger_data_->observation->timestamp_s(j) << " ";

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
    void append_field_data_to_file(std::vector<std::vector<double>> field_data)
    {
        std::ostream_iterator<double> double_iterator(output_file_, " ");

        for (auto data : field_data)
        {
            std::copy(data.begin(), data.end(), double_iterator);
        }
    }

    static void *write(void *instance_pointer)
    {
        ((RobotLogger *)(instance_pointer))->write();
        return nullptr;
    }

    /**
     * @brief Writes everything to the log file.
     *
     * It dumps all the data corresponding to block_size_ number of time indices
     * at one go.
     */
    void write()
    {
        append_header_to_file();

        while (!stop_was_called_ &&
               !(logger_data_->desired_action->length() > 0))
        {
            real_time_tools::Timer::sleep_until_sec(0.1);
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

                append_robot_data_to_file();

                index_ += block_size_;

#ifdef VERBOSE

                // to check whether the data being requested to be logged is in
                // the buffer of the timeseries and inspect effect of delays
                std::cout << "Index trying to access, oldest index in the "
                             "buffer: "
                          << j << ","
                          << logger_data_->observation->oldest_timeindex()
                          << std::endl;

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

    /**
     * @brief Call start() to create the thread for the RobotLogger and start logging!
     *
     * \note
     * Every time you start the logger with the same file name, it will obviously
     * append newer data to the same file. This shouldn't be a problem. But for
     * different log files, specify different file names while starting the logger.
     * 
     * @param filename The name of the log file.
     */
    void start(std::string filename)
    {
        output_file_name_ = filename;
        thread_->create_realtime_thread(&RobotLogger::write, this);
    }

    /**
     * @brief Call stop() when you want to stop logging.
     */
    void stop()
    {
        stop_was_called_ = true;
        thread_->join();
        append_robot_data_to_file();
    }

private:
    std::shared_ptr<real_time_tools::RealTimeThread> thread_;
};

}  // namespace robot_interfaces
