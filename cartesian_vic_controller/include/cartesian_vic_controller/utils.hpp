// Copyright 2024 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
/// \authors: Thibault Poignonec <thibault.poignonec@gmail.com>

#ifndef CARTESIAN_VIC_CONTROLLER__UTILS_HPP_
#define CARTESIAN_VIC_CONTROLLER__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

// C++
#include <map>
#include <memory>
#include <vector>
#include <string>

// ros2 msgs
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cartesian_vic_controller
{

geometry_msgs::msg::Accel AccelToMsg(const Eigen::Matrix<double, 6, 1> & in);
geometry_msgs::msg::Wrench WrenchToMsg(const Eigen::Matrix<double, 6, 1> & in);

template<class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> & e, std_msgs::msg::Float64MultiArray & m);


bool fromMsg(const std_msgs::msg::Float64MultiArray & m, Eigen::Matrix<double, 6, 6> & e);

/**
 * @brief Use to map an upper triangular matrix to a flattened vector
 *        Example:
 *          T = [[1, 2, 3],
 *               [0, 4, 5],
 *               [0, 0, 6]]
 *          flattened_T = [1, 2, 3, 4, 5, 6]
 *
 *          T[1, 2] = flattened_T[flattened_index_from_triangular_index(1, 2)]
 *
 * @param idx_row row index of the triangular matrix
 * @param idx_col column index of the triangular matrix
 * @return unsigned int corresponding flattened matrix index
 */
unsigned int flattened_index_from_triangular_index(unsigned int idx_row, unsigned int idx_col);

}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__UTILS_HPP_
