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
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/loggable.hpp>
#include <robot_interfaces/finger_types.hpp>

namespace robot_interfaces
{

class FingerLogger
{
public:

    FingerTypes::Action applied_action;
    FingerTypes::Action desired_action;
    FingerTypes::Observation observation;
    typedef FingerTypes::Status Status;
    Status status;

    std::shared_ptr<robot_interfaces::RobotData<FingerTypes::Action, FingerTypes::Observation, Status>>
        logger_data_;

    int block_size_;
    long int index_;
    int i, j, loop_iterator_;

    bool stop_was_called_;

    std::ofstream output_file_;
    std::string output_file_name_;
    std::vector<std::string> header;
    std::string temp;

    std::vector<std::string> observation_name_ = observation.get_name();
    std::vector<std::string> applied_action_name_ = applied_action.get_name();
    std::vector<std::string> desired_action_name_ = desired_action.get_name();

    std::vector<std::vector<double>> observation_data_ = observation.get_data();
    std::vector<std::vector<double>> applied_action_data_ = applied_action.get_data();
    std::vector<std::vector<double>> desired_action_data_ = desired_action.get_data();

    FingerLogger(std::shared_ptr<
                     robot_interfaces::RobotData<FingerTypes::Action, FingerTypes::Observation, Status>>
                     robot_data,
                 int block_size,
                 std::string filename)
        : block_size_(block_size),
          stop_was_called_(false),
          logger_data_(robot_data),
          output_file_name_(filename)
    {
        thread_ = std::make_shared<real_time_tools::RealTimeThread>();
    }

    virtual ~FingerLogger()
    {
        stop_was_called_ = true;
        stop();
        thread_->join();

    }

    void get_header()
    {
      //using the newest index inofrmation in logger_data to extract information on the fields to be logged
      applied_action = (*logger_data_->applied_action)[logger_data_->applied_action->newest_timeindex()];
      desired_action = (*logger_data_->desired_action)[logger_data_->desired_action->newest_timeindex()];
      observation = (*logger_data_->observation)[logger_data_->observation->newest_timeindex()];
      status = (*logger_data_->status)[logger_data_->status->newest_timeindex()];

      header.push_back("#");
      header.push_back("[Timestamp]");
      header.push_back("(Current Index)");

      for(i = 0; i < observation_name_.size(); i++)
      {
        for(j = 0; j < observation_data_[i].size(); j ++)
        {
          temp = observation_name_[i] + " " + std::to_string(j);
          header.push_back(temp);
        }
      }

      for(i = 0; i < applied_action_name_.size(); i++)
      {
        for(j = 0; j < applied_action_data_[i].size(); j ++)
        {
          temp = applied_action_name_[i] + " " + std::to_string(j);
          header.push_back(temp);
        }
      }

      for(i = 0; i < desired_action_name_.size(); i++)
      {
        for(j = 0; j < desired_action_data_[i].size(); j ++)
        {
          temp = desired_action_name_[i] + " " + std::to_string(j);
          header.push_back(temp);
        }
      }

      header.push_back("Status");

    }

    static void *write(void *instance_pointer)
    {
        ((FingerLogger *)(instance_pointer))->write();
        return nullptr;
    }

    void write()
    {

        output_file_.open(output_file_name_);
        std::ostream_iterator<std::string> output_iterator(output_file_, " , ");
        get_header();
        std::copy(header.begin(), header.end(), output_iterator);
        output_file_ << std::endl;
        output_file_.close();

        while (!stop_was_called_ &&
               !(logger_data_->desired_action->length() > 0))
        {
            real_time_tools::Timer::sleep_until_sec(0.1);
        }

        index_ = logger_data_->observation->newest_timeindex();

        for (long int t = 0; !stop_was_called_; t++)
        {
            if (index_ + block_size_ ==
                logger_data_->observation->newest_timeindex())
            {
                output_file_.open(output_file_name_, std::ios_base::app);


#ifdef VERBOSE
                auto t1 = std::chrono::high_resolution_clock::now();
#endif

                for (long int j = index_; j < index_ + block_size_; j++)
                {
                    try
                    {
                        std::ostream_iterator<double> output_iterator_(output_file_, " , ");
                        applied_action = (*logger_data_->applied_action)[j];
                        desired_action = (*logger_data_->desired_action)[j];
                        observation = (*logger_data_->observation)[j];
                        status = (*logger_data_->status)[j];

                        observation_data_ = observation.get_data();
                        applied_action_data_ = applied_action.get_data();
                        desired_action_data_ = desired_action.get_data();

                        output_file_ << logger_data_->observation->timestamp_s(j) << " , " << j << " , " ;

                        for(loop_iterator_ = 0; loop_iterator_ < observation_data_.size(); loop_iterator_++)
                        {
                        std::copy(observation_data_[loop_iterator_].begin(), observation_data_[loop_iterator_].end(), output_iterator_);
                        }

                        for(loop_iterator_ = 0; loop_iterator_ < applied_action_data_.size(); loop_iterator_++)
                        {
                          std::copy(applied_action_data_[loop_iterator_].begin(), applied_action_data_[loop_iterator_].end(), output_iterator_);
                        }

                        for(loop_iterator_ = 0; loop_iterator_ < desired_action_data_.size(); loop_iterator_++)
                        {
                          std::copy(desired_action_data_[loop_iterator_].begin(), desired_action_data_[loop_iterator_].end(), output_iterator_);
                        }

                        output_file_ << std::endl;

                        std::cout << j << std::endl;

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

    void start()
    {
        thread_->create_realtime_thread(&FingerLogger::write, this);
    }

    void stop()
    {
      output_file_.open(output_file_name_, std::ios_base::app);
      std::ostream_iterator<double> output_iterator_(output_file_, " , ");

      for(int j = index_; j < logger_data_->observation->newest_timeindex(); j++)
      {
        applied_action = (*logger_data_->applied_action)[j];
        desired_action = (*logger_data_->desired_action)[j];
        observation = (*logger_data_->observation)[j];
        status = (*logger_data_->status)[j];

        observation_data_ = observation.get_data();
        applied_action_data_ = applied_action.get_data();
        desired_action_data_ = desired_action.get_data();

        for(loop_iterator_ = 0; loop_iterator_ < observation_data_.size(); loop_iterator_++)
        {
          std::copy(observation_data_[loop_iterator_].begin(), observation_data_[loop_iterator_].end(), output_iterator_);
        }

        for(loop_iterator_ = 0; loop_iterator_ < applied_action_data_.size(); loop_iterator_++)
        {
          std::copy(applied_action_data_[loop_iterator_].begin(), applied_action_data_[loop_iterator_].end(), output_iterator_);
        }

        for(loop_iterator_ = 0; loop_iterator_ < desired_action_data_.size(); loop_iterator_++)
        {
          std::copy(desired_action_data_[loop_iterator_].begin(), desired_action_data_[loop_iterator_].end(), output_iterator_);
        }

        output_file_ << std::endl;

      }

      output_file_.close();
    }

private:
    std::shared_ptr<real_time_tools::RealTimeThread> thread_;
};

}  // namespace robot_interfaces
