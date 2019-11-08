///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, Max Planck Gesellschaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#pragma once

namespace robot_interfaces
{
/*
 * @brief Contains definitions of the methods to be implemented by all the robot
 * data types.
 */

class Loggable
{
public:
    /*
     * @brief Return the names of the fields in the structure.
     */
    virtual std::vector<std::string> get_name() = 0;

    /*
     * @brief Return the data in the fields of the structure.
     */
    virtual std::vector<std::vector<double>> get_data() = 0;
};

}// namespace robot_interfaces
