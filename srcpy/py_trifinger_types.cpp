/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file
 * \brief Create bindings for TriFinger robot types
 */
#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/pybind_finger.hpp>

using namespace robot_interfaces;

PYBIND11_MODULE(py_trifinger_types, m)
{
    create_blmc_can_robot_python_bindings<TriFingerTypes>(m);
}
