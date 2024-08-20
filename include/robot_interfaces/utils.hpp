/**
 * @file
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD-3-clause
 */
#pragma once

#include <cstdint>

namespace robot_interfaces
{
//! @brief Empty struct that can be used as placeholder.
struct None
{
    template <class Archive>
    void serialize(Archive& archive)
    {
        // need to serialize some dummy value here, as an actual empty type will
        // cause trouble for our shared_memory implementation
        uint8_t dummy = 0;
        archive(dummy);
    }
};
}  // namespace robot_interfaces
