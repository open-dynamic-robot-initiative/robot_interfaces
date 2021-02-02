/**
 * @file
 * @brief Observation of a Finger robot.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <memory>

#include "robot_backend.hpp"
#include "robot_data.hpp"
#include "robot_frontend.hpp"
#include "robot_log_entry.hpp"
#include "robot_log_reader.hpp"
#include "robot_logger.hpp"

namespace robot_interfaces
{
template <typename Action_t, typename Observation_t>
struct RobotInterfaceTypes
{
    typedef Action_t Action;
    typedef Observation_t Observation;

    typedef RobotBackend<Action, Observation> Backend;
    typedef std::shared_ptr<Backend> BackendPtr;

    typedef RobotData<Action, Observation> BaseData;
    typedef std::shared_ptr<BaseData> BaseDataPtr;
    typedef SingleProcessRobotData<Action, Observation> SingleProcessData;
    typedef std::shared_ptr<SingleProcessData> SingleProcessDataPtr;
    typedef MultiProcessRobotData<Action, Observation> MultiProcessData;
    typedef std::shared_ptr<MultiProcessData> MultiProcessDataPtr;

    typedef RobotFrontend<Action, Observation> Frontend;
    typedef std::shared_ptr<Frontend> FrontendPtr;

    typedef RobotLogEntry<Action, Observation> LogEntry;
    typedef RobotLogger<Action, Observation> Logger;
    typedef RobotBinaryLogReader<Action, Observation> BinaryLogReader;
};

}  // namespace robot_interfaces
