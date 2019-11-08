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

#include <robot_interfaces/infix_iterator.h>
#include <mpi_cpp_tools/basic_tools.hpp>
#include <mpi_cpp_tools/dynamical_systems.hpp>
#include <mpi_cpp_tools/math.hpp>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/loggable.hpp>

namespace robot_interfaces
{
  /**
   * @brief To log data from *any* robot (real, simulated, fake).
   *
   * The logger logs the timestamp, the time index, and the values of every
   * observation, action, and status variable. Further loggable data structures
   * can be added to the logger, which *must* derive from class Loggable.
   *
   * @tparam Action
   * @tparam Observation
   * @tparam Status
   */
template <typename Action, typename Observation, typename Status>
class RobotLogger
{
public:
    static_assert(std::is_base_of<Loggable, Action>::value,
                  "Action must derive from Loggable");
    static_assert(std::is_base_of<Loggable, Observation>::value,
                  "Observation must derive from Loggable");
    static_assert(std::is_base_of<Loggable, Status>::value,
                  "Status must derive from Loggable");

    /*
    * Currently, the level of generalisability of the logger is that we know
    * the following four structures exist for *any* robot whose data is to be
    * logged.
    */

    Action applied_action;
    Action desired_action;
    Observation observation;
    Status status;

    std::shared_ptr<robot_interfaces::RobotData<Action, Observation, Status>>
        logger_data_;

    std::vector<std::string> header_;

    int block_size_;
    long int index_;

    bool stop_was_called_;

    std::ofstream output_file_;
    std::string output_file_name_;

    RobotLogger(std::shared_ptr<
                    robot_interfaces::RobotData<Action, Observation, Status>>
                    robot_data,
                int block_size)
        : block_size_(block_size),
          stop_was_called_(false),
          logger_data_(robot_data)
    {
        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
    }

    virtual ~RobotLogger()
    {
        stop_was_called_ = true;
        stop();
        thread_->join();
    }

    std::vector<std::string> get_header()
    {
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

        std::string temp;
        int i, j;

        // using the newest index information in logger_data to extract
        // information on the fields to be logged
        applied_action =
            (*logger_data_->applied_action)[logger_data_->applied_action
                                                ->newest_timeindex()];
        desired_action =
            (*logger_data_->desired_action)[logger_data_->desired_action
                                                ->newest_timeindex()];
        observation =
            (*logger_data_
                  ->observation)[logger_data_->observation->newest_timeindex()];
        status =
            (*logger_data_->status)[logger_data_->status->newest_timeindex()];

        header_.push_back("#");
        header_.push_back("[Timestamp]");
        header_.push_back("(Time Index)");

        append_name_to_header(status_name, status_data);
        append_name_to_header(observation_name, observation_data);
        append_name_to_header(applied_action_name, applied_action_data);
        append_name_to_header(desired_action_name, desired_action_data);

        return header_;
    }

    void append_name_to_header(std::vector<std::string> field_name,
                               std::vector<std::vector<double>> field_data)
    {
        int i, j;
        std::string temp;

        for (i = 0; i < field_name.size(); i++)
        {
          if(field_data[i].size() == 1)
          {
            temp = field_name[i];
            header_.push_back(temp);
          }
          else
          {
            for (j = 0; j < field_data[i].size(); j++)
            {
                temp = field_name[i] + " " + std::to_string(j);
                header_.push_back(temp);
            }
          }
        }
    }

    //TODO: Using the infix_iterator results in a blank log file.
    void append_header_to_file()
    {
        output_file_.open(output_file_name_);

        std::ostream_iterator<std::string> mid_iterator_string(output_file_,
                                                               " , ");
        std::ostream_iterator<std::string> end_iterator_string(output_file_,
                                                               "\n");

        header_ = get_header();

        std::copy(header_.begin(), header_.end() - 1, mid_iterator_string);
        std::copy(header_.end() - 1, header_.end(), end_iterator_string);

        output_file_.close();
    }

    //TODO: fix the working of the infix_iterator and move the redundant writing
    //part in a separate method.
    void append_robot_data_to_file()
    {
        std::ostream_iterator<double> mid_iterator_double(output_file_, " , ");
        std::ostream_iterator<double> end_iterator_double(output_file_, "\n");

        std::vector<std::vector<double>> observation_data =
            observation.get_data();
        std::vector<std::vector<double>> applied_action_data =
            applied_action.get_data();
        std::vector<std::vector<double>> desired_action_data =
            desired_action.get_data();
        std::vector<std::vector<double>> status_data = status.get_data();

        int k;

        for (long int j = index_;
             j < std::min(index_ + block_size_,
                          logger_data_->observation->newest_timeindex());
             j++)
        {
            try
            {
                applied_action = (*logger_data_->applied_action)[j];
                desired_action = (*logger_data_->desired_action)[j];
                observation = (*logger_data_->observation)[j];
                status = (*logger_data_->status)[j];

                status_data = status.get_data();
                observation_data = observation.get_data();
                applied_action_data = applied_action.get_data();
                desired_action_data = desired_action.get_data();

                output_file_ << logger_data_->observation->timestamp_s(j)
                             << " , " << j << " , ";

                for (k = 0; k < status_data.size(); k++)
                {
                    std::copy(status_data[k].begin(),
                              status_data[k].end(),
                              mid_iterator_double);
                }

                for (k = 0; k < observation_data.size(); k++)
                {
                    std::copy(observation_data[k].begin(),
                              observation_data[k].end(),
                              mid_iterator_double);
                }

                for (k = 0; k < applied_action_data.size(); k++)
                {
                    std::copy(applied_action_data[k].begin(),
                              applied_action_data[k].end(),
                              mid_iterator_double);
                }

                for (k = 0; k < desired_action_data.size(); k++)
                {
                    std::copy(desired_action_data[k].begin(),
                              desired_action_data[k].end(),
                              mid_iterator_double);
                }

                std::copy(
                    desired_action_data[desired_action_data.size() - 1].end() -
                        1,
                    desired_action_data[desired_action_data.size() - 1].end(),
                    end_iterator_double);
            }

            catch (const std::exception &e)
            {
                std::cout << "Trying to access index older than the "
                             "oldest! Skipping ahead."
                          << std::endl;
            }
        }
    }

    static void *write(void *instance_pointer)
    {
        ((RobotLogger *)(instance_pointer))->write();
        return nullptr;
    }

    void write()
    {
        append_header_to_file();

        output_file_.open(output_file_name_, std::ios_base::app);

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

                        }

                        for(k = 0; k < desired_action_data.size(); k++)
                        {
                          std::copy(desired_action_data[k].begin(), desired_action_data[k].end() - 1, mid_iterator_);
                          std::copy(desired_action_data[k].end() - 1, desired_action_data[k].end(), end_iterator_);
                        }

                        // output_file_ << std::endl;
                    }

                    catch (const std::exception &e)
                    {
                        std::cout << "Trying to access index older than the "
                                     "oldest! Skipping ahead."
                                  << std::endl;
                    }
                  }

                    index_ += block_size_;

                    output_file_.close();

// to check whether the data being requested to be logged is in the buffer of
// the timeseries and inspect effect of delays
#ifdef VERBOSE
                std::cout << "Index trying to access, oldest index in the "
                             "buffer: "
                          << j << ","
                          << logger_data_->observation->oldest_timeindex()
                          << std::endl;
#endif

// to print the time taken for one block of data to be logged.
#ifdef VERBOSE
                auto t2 = std::chrono::high_resolution_clock::now();
                auto duration =
                    std::chrono::duration_cast<std::chrono::microseconds>(t2 -
                                                                          t1)
                        .count();

                std::cout << "Time taken for one block of data to be logged: "
                          << duration << std::endl;
#endif
            }
        }
    }

    void start(std::string filename)
    {
        output_file_name_ = filename;
        thread_->create_realtime_thread(&RobotLogger::write, this);
    }

    void stop()
    {
        output_file_.open(output_file_name_, std::ios_base::app);

        append_robot_data_to_file();

        output_file_.close();
    }

private:
    std::shared_ptr<real_time_tools::RealTimeThread> thread_;
};

}// namespace robot_interfaces
