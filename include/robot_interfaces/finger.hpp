///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <math.h>

#include "mpi_cpp_tools/basic_tools.hpp"
#include "mpi_cpp_tools/dynamical_systems.hpp"
#include "mpi_cpp_tools/math.hpp"

#include "real_time_tools/thread.hpp"
#include "real_time_tools/threadsafe/threadsafe_timeseries.hpp"
#include "real_time_tools/timer.hpp"
#include <real_time_tools/process_manager.hpp>

namespace robot_interfaces {

// |------ t = 0 ------|------ t = 1 ------|
// |----- action0 -----|----- action1 -----|
// o                   o                   o
// b                   b                   b
// s                   s                   s
// 0                   1                   2

template <typename Action, typename Observation> class Robot {
public:
  Robot(const double &max_action_duration_s,
        const double &max_inter_action_duration_s)
      : max_action_duration_s_(max_action_duration_s),
        max_inter_action_duration_s_(max_inter_action_duration_s),
        is_shutdown_(false), action_start_logger_(1000),
        action_end_logger_(1000) {
    thread_ = std::make_shared<real_time_tools::RealTimeThread>();
    thread_->create_realtime_thread(&Robot::loop, this);
  }

  ~Robot() {
    shutdown_and_stop_thread();
    thread_->join();
  }

  double get_max_inter_action_duration_s() {
    return max_inter_action_duration_s_;
  }

  virtual Action
  apply_action_and_check_timing(const Action &desired_action) final {
    if (is_shutdown_) {
      std::cout << "you tried to apply action, but robot has been shut down"
                << std::endl;
      return desired_action;
    }
    action_start_logger_.append(true);
    Action applied_action = apply_action(desired_action);
    action_end_logger_.append(true);
    return applied_action;
  }

protected:
  virtual Action apply_action(const Action &desired_action) = 0;

public:
  virtual Observation get_latest_observation() = 0;

protected:
  virtual void shutdown_and_stop_thread() final {
    if (!is_shutdown_) {
      is_shutdown_ = true;
      shutdown();
    }
  }
  virtual void shutdown() = 0;

private:
  void loop() {
    real_time_tools::set_cpu_dma_latency(0);

    while (!is_shutdown_ && !action_start_logger_.wait_for_timeindex(0, 0.1)) {
    }

    for (size_t t = 0; !is_shutdown_; t++) {
      bool action_has_ended_on_time =
          action_end_logger_.wait_for_timeindex(t, max_action_duration_s_);
      if (!action_has_ended_on_time) {
        std::cout << "action did not end on time, shutting down." << std::endl;
        exit(-1); // temp
        shutdown_and_stop_thread();
        return;
      }

      bool action_has_started_on_time = action_start_logger_.wait_for_timeindex(
          t + 1, max_inter_action_duration_s_);
      if (!action_has_started_on_time) {
        std::cout << "action did not start on time, shutting down."
                  << std::endl;
        exit(-1); // temp
        shutdown_and_stop_thread();
        return;
      }
    }
  }
  static void *loop(void *instance_pointer) {
    ((Robot *)(instance_pointer))->loop();
    return nullptr;
  }

private:
  double max_action_duration_s_;
  double max_inter_action_duration_s_;

  bool is_shutdown_; // todo: should be atomic

  real_time_tools::ThreadsafeTimeseries<bool> action_start_logger_;
  real_time_tools::ThreadsafeTimeseries<bool> action_end_logger_;

  std::shared_ptr<real_time_tools::RealTimeThread> thread_;
};

template <typename Action, typename Observation, typename Status>
class RobotData {
public:
  template <typename Type>
  using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;
  typedef Timeseries<int>::Index TimeIndex; // \TODO this is not quite clean
                                            // because we should not have to
                                            // specify a type here.
  template <typename Type> using Ptr = std::shared_ptr<Type>;

  RobotData(size_t history_length = 1000, bool use_shared_memory = false,
            std::string shared_memory_address = "") {
    if (use_shared_memory) {
      std::cout << "shared memory robot data is not implemented yet"
                << std::endl;
      exit(-1);

      // todo: here we should check if the shared memory at that
      // address already exists, otherwise we create it.
      // we will also have to update timeseries such as to handle serialization
      // internally (it will simply assume that the templated class has a
      // method called serialize() and from_serialized())
    } else {
      desired_action = std::make_shared<Timeseries<Action>>(history_length);
      applied_action = std::make_shared<Timeseries<Action>>(history_length);
      observation = std::make_shared<Timeseries<Observation>>(history_length);
      status = std::make_shared<Timeseries<Status>>(history_length);
    }
  }

public:
  Ptr<Timeseries<Action>> desired_action;
  Ptr<Timeseries<Action>> applied_action;
  Ptr<Timeseries<Observation>> observation;
  Ptr<Timeseries<Status>> status;
};

template <typename Action, typename Observation> class RobotServer {
public:
  struct Status {
    bool received_action;
  };
  // add parameter: n_max_repeat_of_same_action
  RobotServer(std::shared_ptr<Robot<Action, Observation>> robot,
              RobotData<Action, Observation, Status> robot_data)
      : robot_(robot), robot_data_(robot_data), destructor_was_called_(false) {
    thread_ = std::make_shared<real_time_tools::RealTimeThread>();
    thread_->create_realtime_thread(&RobotServer::loop, this);
  }

  virtual ~RobotServer() {
    destructor_was_called_ = true;
    thread_->join();
  }

private:
  std::shared_ptr<Robot<Action, Observation>> robot_;
  RobotData<Action, Observation, Status> robot_data_;
  bool destructor_was_called_; // should be atomic

  std::vector<real_time_tools::Timer> timers_;

  std::shared_ptr<real_time_tools::RealTimeThread> thread_;

  // control loop ------------------------------------------------------------
  static void *loop(void *instance_pointer) {
    ((RobotServer *)(instance_pointer))->loop();
    return nullptr;
  }
  void loop() {

    timers_.resize(10);
    real_time_tools::set_cpu_dma_latency(0);

    while (!destructor_was_called_ &&
           !robot_data_.desired_action->wait_for_timeindex(0, 0.1)) {
    }

    for (long int t = 0; !destructor_was_called_; t++) {

      // todo: figure out latency stuff!! open /dev/cpu_dma_latency: Permission
      // denied

      timers_[0].tac_tic();

      timers_[6].tic();
      Observation observation = robot_->get_latest_observation();
      timers_[6].tac();

      timers_[1].tic();
      robot_data_.observation->append(observation); // todo: for some reason
                                                    // this smetimes takes more
                                                    // than 2 ms
      timers_[1].tac();

      timers_[2].tic();
      if (std::isfinite(robot_->get_max_inter_action_duration_s()) &&
          robot_data_.desired_action->newest_timeindex() < t) {
        /// TODO: we should somehow log if a set has been missed
        robot_data_.desired_action->append(
            robot_data_.desired_action->newest_element());
      }
      timers_[2].tac();

      timers_[3].tic();
      Action desired_action = (*robot_data_.desired_action)[t];
      timers_[3].tac();
      timers_[4].tic();
      Action applied_action =
          robot_->apply_action_and_check_timing(desired_action);
      timers_[4].tac();
      timers_[5].tic();
      robot_data_.applied_action->append(applied_action);
      timers_[5].tac();

      // if (t % 5000 == 0) {
      //   for (size_t i = 0; i < 7; i++) {
      //     std::cout << i << " --------------------------------------"
      //               << std::endl;
      //     timers_[i].print_statistics();
      //   }
      // }
    }
  }
};

class Finger {
public:
  enum JointIndexing { base, center, tip, joint_count };

  template <typename Type>
  using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;
  typedef Timeseries<int>::Index TimeIndex;
  typedef Timeseries<int>::Timestamp TimeStamp;

  typedef Eigen::Vector3d Vector;
  typedef Vector Action;
  struct Observation {
    Vector angle;
    Vector velocity;
    Vector torque;
  };

  typedef RobotServer<Action, Observation>::Status Status;

  /// TODO: remove default values!!!
  Finger(std::shared_ptr<Robot<Action, Observation>> robot,
         const double &expected_step_duration_ms = 1.0,
         const double &step_duration_tolerance_ratio = 5.0,
         const bool &is_realtime = true) {

    robot_server_ =
        std::make_shared<RobotServer<Action, Observation>>(robot, robot_data_);
  }

  Observation get_observation(const TimeIndex &t) {
    return (*robot_data_.observation)[t];
  }
  Action get_desired_action(const TimeIndex &t) {
    return (*robot_data_.desired_action)[t];
  }
  Action get_applied_action(const TimeIndex &t) {
    return (*robot_data_.applied_action)[t];
  }
  TimeStamp get_time_stamp_ms(const TimeIndex &t) {
    return robot_data_.observation->timestamp_ms(t);
  }
  TimeIndex get_current_timeindex() {
    return robot_data_.observation->newest_timeindex();
  }

  TimeIndex append_desired_action(const Action &desired_action) {
    if (robot_data_.desired_action->length() ==
            robot_data_.desired_action->max_length() &&
        robot_data_.desired_action->oldest_timeindex() ==
            get_current_timeindex()) {
      std::cout << "you have been appending actions too fast, waiting for "
                   "RobotServer to catch up with executing actions."
                << std::endl;
      wait_until_timeindex(robot_data_.desired_action->oldest_timeindex() + 1);
    }

    robot_data_.desired_action->append(desired_action);
    return robot_data_.desired_action->newest_timeindex();
  }

  void wait_until_timeindex(const TimeIndex &t) {
    robot_data_.observation->timestamp_ms(t);
  }

private:
  RobotData<Action, Observation, Status> robot_data_;
  std::shared_ptr<RobotServer<Action, Observation>> robot_server_;
};

} // namespace robot_interfaces
