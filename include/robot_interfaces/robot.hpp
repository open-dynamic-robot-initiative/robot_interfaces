///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <robot_interfaces/monitored_robot_driver.hpp>
#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_frontend.hpp>
#include <type_traits>

namespace robot_interfaces
{
/**
 * @brief RobotFrontend that construct and encapsulates
 * its related RobotBackend. It also construct and starts
 * the robot driver.
 */
template <typename Action,
          typename Observation,
          typename Driver,
          typename Data = SingleProcessRobotData<Action, Observation>>
class Robot : public RobotFrontend<Action, Observation>
{
    typedef std::shared_ptr<Data> DataPtr;
    typedef std::shared_ptr<Driver> RobotDriverPtr;

public:
    /**
     * @param max_action_duration_s See MonitoredRobotDriver.
     * @param max_inter_action_duration_s See MonitoredRobotDriver.
     * @param args Arguments required to instantiate the driver
     * by this constructor if not provided.
     */
    template <typename... Args>
    Robot(double max_action_duration_s,
          double max_inter_action_duration_s,
          Args... args)
        : RobotFrontend<Action, Observation>(std::make_shared<Data>()),
          driver_ptr_(std::make_shared<Driver>(args...)),
          backend_(
              std::make_shared<
                  robot_interfaces::MonitoredRobotDriver<Driver>>(
                  driver_ptr_,
                  max_action_duration_s,
                  max_inter_action_duration_s),
              this->robot_data_)
    {
        // compile time checking template Driver inherate from RobotDriver
        static_assert(
            std::is_base_of<RobotDriver<Action, Observation>, Driver>::value,
            "template Driver must be a subclass of "
            "robot_interfaces::RobotDriver");
    }

    /**
     * initialize the backend
     */
    void initialize()
    {
        backend_.initialize();
    }

    /**
     * return the data shared by the frontend and the backend.
     */
    const Data& get_data() const
    {
        return *(this->robot_data_);
    }

private:
    RobotDriverPtr driver_ptr_;
    RobotBackend<Action, Observation> backend_;
};
}  // namespace robot_interfaces
