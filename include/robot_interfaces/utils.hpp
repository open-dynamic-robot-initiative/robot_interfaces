/**
 * @file
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD-3-clause
 */
#pragma once

namespace robot_interfaces
{
//! @brief Empty struct that can be used as placeholder.
struct None
{
    template <class Archive>
    void serialize(Archive&)
    {
        // nothing to do here
    }
};
}  // namespace robot_interfaces
