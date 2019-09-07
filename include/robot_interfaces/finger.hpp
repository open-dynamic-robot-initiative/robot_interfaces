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
  /// todo: this should return the actually applied action
  /// todo: we should probably have an initialize function here.
public:
  // virtual void start() = 0;
  // virtual void restart / initialize
  // virtual void stop() = 0;
  virtual Action apply_action(const Action &desired_action) = 0;
  virtual Observation get_latest_observation() = 0;
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

  typedef long unsigned
      TimeIndex; // \TODO this should be defined based on timeseries

  RobotServer(std::shared_ptr<Robot<Action, Observation>> robot,
              RobotData<Action, Observation, Status> robot_data,
              const double &expected_step_duration_ms,
              const double &step_duration_tolerance_ratio,
              const bool &is_realtime)
      : robot_(robot), robot_data_(robot_data), destructor_was_called_(false),
        expected_step_duration_ms_(expected_step_duration_ms),
        step_duration_tolerance_ratio_(step_duration_tolerance_ratio),
        is_realtime_(is_realtime) {

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
  bool destructor_was_called_;

  /// todo: i think we can parametrize this a bit more elegantly
  double expected_step_duration_ms_;
  double step_duration_tolerance_ratio_;
  bool is_realtime_;

  std::shared_ptr<real_time_tools::RealTimeThread> thread_;

  // control loop ------------------------------------------------------------
  static void *loop(void *instance_pointer) {
    ((RobotServer *)(instance_pointer))->loop();
    return nullptr;
  }

  void loop() {
    real_time_tools::set_cpu_dma_latency(0);

    real_time_tools::Timer timer;

    for (TimeIndex t = 0; !destructor_was_called_; t++) {

      // todo: figure out latency stuff!! open /dev/cpu_dma_latency: Permission
      // denied
      //   if (t % 1000 == 0) {
      //     timer.print_statistics();
      //   }
      robot_data_.observation->append(robot_->get_latest_observation());

      if (is_realtime_ &&
          (robot_data_.desired_action->length() == 0 ||
           robot_data_.desired_action->newest_timeindex() < t)) {
        if (robot_data_.desired_action->length() == 0) {
          robot_data_.desired_action->append(Action::Zero());
        } else {
          /// TODO: we should somehow log if a set has been missed
          robot_data_.desired_action->append(
              robot_data_.desired_action->newest_element());
        }
      }

      Action desired_action = (*robot_data_.desired_action)[t];
      timer.tic();
      Action applied_action = robot_->apply_action(desired_action);
      timer.tac();
      robot_data_.applied_action->append(applied_action);

      if (t >= 1) {
        check_timing(robot_data_.observation->timestamp_ms(t) -
                     robot_data_.observation->timestamp_ms(t - 1));
      }
    }
  }

  // helper functions --------------------------------------------------------
  void check_timing(const double &actual_step_duration_ms) {
    double max_step_duration_ms =
        expected_step_duration_ms_ * step_duration_tolerance_ratio_;
    double min_step_duration_ms =
        expected_step_duration_ms_ / step_duration_tolerance_ratio_;

    if (is_realtime_ && (actual_step_duration_ms > max_step_duration_ms ||
                         actual_step_duration_ms < min_step_duration_ms)) {
      std::ostringstream oss;
      oss << "control loop did not run at expected rate "
             "did you provide actions fast enough? "
          << std::endl
          << "expected step duration: " << expected_step_duration_ms_
          << std::endl
          << "actual step duration: " << actual_step_duration_ms << std::endl
          << "tolearance ratio: " << step_duration_tolerance_ratio_
          << std::endl;
      throw std::runtime_error(oss.str());
    }
  }
};

class Finger {
public:
  enum JointIndexing { base, center, tip, joint_count };

  template <typename Type>
  using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;

  typedef Timeseries<int>::Index TimeIndex; // \TODO this is not quite clean
                                            // because we should not have to
                                            // specify a type here.
  typedef Timeseries<int>::Timestamp TimeStamp;

  typedef Eigen::Vector3d Vector;

  typedef Eigen::Vector3d Action;

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

    robot_server_ = std::make_shared<RobotServer<Action, Observation>>(
        robot, robot_data_, expected_step_duration_ms,
        step_duration_tolerance_ratio, is_realtime);
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
