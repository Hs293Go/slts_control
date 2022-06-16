// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Core>

namespace common {
struct SLTSProperty {
  double uav_mass;
  double pld_mass;
  double cable_length;
};

struct SLTSStates {
  Eigen::Vector3d pld_abs_pos;
  Eigen::Vector2d pld_rel_pos;
  Eigen::Vector3d pld_abs_vel;
  Eigen::Vector2d pld_rel_vel;
};
}  // namespace common

#endif  // COMMON_H
