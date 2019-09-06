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

namespace robot_interfaces {

// |------ t = 0 ------|------ t = 1 ------|
// |----- action0 -----|----- action1 -----|
// o                   o                   o
// b                   b                   b
// s                   s                   s
// 0                   1                   2

// action_1 -> action_2 ...
//         \   A
//          \ /
//           X
//          / \
//         /   V
// observ_1 -> observ_2 ...

template <typename Action, typename Observation> class Robot {
  virtual Action apply_action(Action desired_action) = 0;
  virtual Observation get_observation() = 0;
};

template <typename Action, typename Observation, typename Status>
class RobotData {
public:
  template <typename Type>
  using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;
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

  RobotServer(Robot<Action, Observation> robot,
              RobotData<Action, Observation, Status> robot_data,
              const double &expected_step_duration_ms = 1.0,
              const double &step_duration_tolerance_ratio = 2.0,
              const bool &is_realtime = true)
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
  Robot<Action, Observation> robot_;
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

  void loop() {}
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

  /// TODO: remove default values!!!
  Finger(const double &expected_step_duration_ms = 1.0,
         const double &step_duration_tolerance_ratio = 2.0,
         const bool &is_realtime = true)
      : expected_step_duration_ms_(expected_step_duration_ms),
        step_duration_tolerance_ratio_(step_duration_tolerance_ratio),
        is_realtime_(is_realtime), is_paused_(false),
        destructor_was_called_(false) {

    size_t history_length = 1000;
    desired_action_ = std::make_shared<Timeseries<Action>>(history_length);
    applied_action_ = std::make_shared<Timeseries<Action>>(history_length);
    observation_ = std::make_shared<Timeseries<Observation>>(history_length);

    thread_ = std::make_shared<real_time_tools::RealTimeThread>();
    thread_->create_realtime_thread(&Finger::loop, this);
  }

  virtual ~Finger() {
    // std::cout << "desctructor called" << std::endl;
    destructor_was_called_ = true;
    thread_->join();
  }

  Observation get_observation(const TimeIndex &t) { return (*observation_)[t]; }
  Action get_desired_action(const TimeIndex &t) {
    return (*desired_action_)[t];
  }
  Action get_applied_action(const TimeIndex &t) {
    return (*applied_action_)[t];
  }
  TimeStamp get_time_stamp_ms(const TimeIndex &t) {
    return observation_->timestamp_ms(t);
  }

  TimeIndex append_desired_action(const Action &desired_action) {
    // TODO: we should make sure not so many actions are appended
    // that we do not have enough history containing all actions to
    // be applied.
    is_paused_ = false;
    desired_action_->append(desired_action);
    return desired_action_->newest_timeindex();
  }

  void wait_until_time_index(const TimeIndex &t) {
    observation_->timestamp_ms(t);
  }
  TimeIndex current_time_index() { return observation_->newest_timeindex(); }
  void pause() {
    is_paused_ = true;
    desired_action_->append(Action::Zero());
  }

protected:
  virtual Observation get_latest_observation() = 0;

protected:
  virtual void apply_action(const Action &action) = 0;
  bool destructor_was_called_;
  std::shared_ptr<real_time_tools::RealTimeThread> thread_;

private:
  std::shared_ptr<Timeseries<Action>> desired_action_;
  std::shared_ptr<Timeseries<Action>> applied_action_;
  std::shared_ptr<Timeseries<Observation>> observation_;

  bool is_paused_;

  double expected_step_duration_ms_;
  double step_duration_tolerance_ratio_;
  bool is_realtime_;

  // control loop ------------------------------------------------------------
  static void *loop(void *instance_pointer) {
    ((Finger *)(instance_pointer))->loop();
    return nullptr;
  }
  void loop() {
    // TODO: there is a slight problem here: this thread
    // may start running before child class is created, and
    // hence attempt to call nonexistant function.
    for (TimeIndex t = 0; true; t++) {
      if (destructor_was_called_ == true) {
        return;
      }

      if (is_realtime_ && (desired_action_->length() == 0 ||
                           desired_action_->newest_timeindex() < t)) {
        if (is_paused_ || desired_action_->length() == 0) {
          desired_action_->append(Action::Zero());
        } else {
          /// TODO: we should somehow log if a set has been missed
          desired_action_->append(desired_action_->newest_element());
        }
      }

      Action desired_action = (*desired_action_)[t];
      Observation observation = get_latest_observation();
      Action applied_action =
          compute_applied_action(desired_action, observation);
      applied_action_->append(applied_action);
      observation_->append(observation);

      // applied_action_->append(constrain_action();

      apply_action((*applied_action_)[t]);

      if (t >= 1) {
        check_timing(observation_->timestamp_ms(t) -
                     observation_->timestamp_ms(t - 1));
      }
    }

    // TODO: we have to make sure this loop exits properly at destruction
    // time!!
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

protected:
  virtual Action compute_applied_action(const Action &desired_action,
                                        const Observation &observation) {
    // Vector kd(0.04, 0.08, 0.02);

    Vector kd(0.08, 0.08, 0.04);
    double max_torque = 0.36;
    Action applied_action = mct::clamp(desired_action, -max_torque, max_torque);
    applied_action = applied_action - kd.cwiseProduct(observation.velocity);
    applied_action = mct::clamp(applied_action, -max_torque, max_torque);
    return applied_action;
  }
  // todo: this should go away
  double max_torque_;

public:
  Vector get_max_torques() const { return max_torque_ * Vector::Ones(); }
};

} // namespace robot_interfaces
