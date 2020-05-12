///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>
#include <vector>

#include "sensor_data.hpp"

namespace robot_interfaces
{
template <typename Observation>
class SensorLogger
{
public:
    typedef std::shared_ptr<SensorData<Observation>> DataPtr;

    SensorLogger(DataPtr sensor_data) : sensor_data_(sensor_data)
    {
    }

    void reset()
    {

    }

    void save(const std::string &filename) const
    {

    }

private:
    DataPtr sensor_data_;
    std::vector<Observation> buffer_;
};

}  // namespace robot_interfaces
