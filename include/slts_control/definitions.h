// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Eigen/Dense>

#include "slts_control/integral.h"

namespace control {
struct RobustTrackerParams {
  double k_pos_err = 0.24;
  double k_gen_trans_err = 0.10;
  double k_trim = 5;
  double k_swing = 0.15;
  double total_de_gain = 0.5;
  Eigen::VectorXd total_de_ub{Eigen::Vector3d::Constant(10.0)};
  Eigen::VectorXd total_de_lb{Eigen::Vector3d::Constant(-10.0)};

  double uav_de_gain = 0.9;
  Eigen::VectorXd uav_de_ub{Eigen::Vector3d::Constant(10.0)};
  Eigen::VectorXd uav_de_lb{Eigen::Vector3d::Constant(-10.0)};

  double k_filter_leak = 0.4;
  double k_filter_gain = 0.2;
  Eigen::VectorXd filt_cross_feeding_ub{Eigen::Vector3d::Constant(10.0)};
  Eigen::VectorXd filt_cross_feeding_lb{Eigen::Vector3d::Constant(-10.0)};
};

struct RobustTrackerInitialConditions {
  Eigen::Vector3d uav_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d uav_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector2d pld_rel_pos{Eigen::Vector2d::Zero()};
  Eigen::Vector2d pld_rel_vel{Eigen::Vector2d::Zero()};
};

namespace details {
struct DisturbanceEstimate {
  double gain;
  math::Integral<Eigen::Vector3d> integral;
  Eigen::Vector3d value;
};

}  // namespace details

}  // namespace control

#endif  // DEFINITIONS_H