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

#include <cartesian_vic_controller/utils.hpp>

namespace cartesian_vic_controller
{

geometry_msgs::msg::Accel AccelToMsg(const Eigen::Matrix<double, 6, 1> & in)
{
  geometry_msgs::msg::Accel msg;
  msg.linear.x = in[0];
  msg.linear.y = in[1];
  msg.linear.z = in[2];
  msg.angular.x = in[3];
  msg.angular.y = in[4];
  msg.angular.z = in[5];
  return msg;
}

geometry_msgs::msg::Wrench WrenchToMsg(const Eigen::Matrix<double, 6, 1> & in)
{
  geometry_msgs::msg::Wrench msg;
  msg.force.x = in[0];
  msg.force.y = in[1];
  msg.force.z = in[2];
  msg.torque.x = in[3];
  msg.torque.y = in[4];
  msg.torque.z = in[5];
  return msg;
}

bool fromMsg(const std_msgs::msg::Float64MultiArray & m, Eigen::Matrix<double, 6, 6> & e)
{
  if (m.data.size() != 36) {
    return false;
  }
  /*
  // TODO(tpoignonec): check layout validity !!! (could be flattened or not...)
  size_t size_in_msg = m.layout.dim[0].size;
  assert(m.layout.dim.size() == 2);
  assert(m.layout.dim[0].stride == e.rows() * e.cols());
  assert(m.layout.dim[0].stride == e.rows() * e.cols());
  assert(m.layout.dim[1].stride == e.cols());
  assert(m.layout.dim[1].stride == e.size());
  */
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i) {
    for (int j = 0; j < e.cols(); ++j) {
      e(i, j) = m.data[ii++];
    }
  }
  return true;
}

unsigned int flattened_index_from_triangular_index(unsigned int idx_row, unsigned int idx_col)
{
  unsigned int i = idx_row;
  unsigned int j = idx_col;
  if (idx_col < idx_row) {
    i = idx_col;
    j = idx_row;
  }
  return i * (i - 1) / 2 + j;
}

}  // namespace cartesian_vic_controller
