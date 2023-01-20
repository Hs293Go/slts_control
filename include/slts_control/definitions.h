// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SLTS_CONTROL_DEFINITIONS_H_
#define SLTS_CONTROL_DEFINITIONS_H_

#include <Eigen/Dense>

#include "slts_control/integral.h"

namespace control {

struct NumDiffMode {
  struct Forward_t {
    Forward_t() = default;
  };

  struct Backward_t {
    Backward_t() = default;
  };

  static constexpr Forward_t Forward{};

  static constexpr Backward_t Backward{};
};

enum class Frame { kENU, kNED };

enum class SLTSControllerStatus {
  kIcUnset,
  kParamsUnset,
  kRelativeVelocityFault,
  kTimeStepFault,
  kRunning
};

struct SLTSControllerParams {
  Frame frame{Frame::kNED};
  double uav_mass;
  double pld_mass;
  double cable_length;

  Eigen::Vector3d k_pos_err{Eigen::Vector3d::Constant(0.24)};
  Eigen::Vector3d k_gen_trans_err{Eigen::Vector3d::Constant(0.10)};
  double k_trim = 5;
  double k_swing = 0.15;
  Eigen::Vector3d total_de_gain{Eigen::Vector3d::Constant(0.5)};
  Eigen::Vector3d total_de_ub{Eigen::Vector3d::Constant(10.0)};
  Eigen::Vector3d total_de_lb{Eigen::Vector3d::Constant(-10.0)};

  Eigen::Vector3d uav_de_gain{Eigen::Vector3d::Constant(0.9)};
  Eigen::Vector3d uav_de_ub{Eigen::Vector3d::Constant(10.0)};
  Eigen::Vector3d uav_de_lb{Eigen::Vector3d::Constant(-10.0)};

  double k_filter_leak = 0.4;
  double k_filter_gain = 0.2;
  Eigen::Vector3d filt_cross_feeding_ub{Eigen::Vector3d::Constant(10.0)};
  Eigen::Vector3d filt_cross_feeding_lb{Eigen::Vector3d::Constant(-10.0)};

  Eigen::Vector3d pld_rel_vel_ub{Eigen::Vector3d::Constant(0.5)};
  Eigen::Vector3d pld_rel_vel_lb{Eigen::Vector3d::Constant(-0.5)};
};

struct SLTSControllerInitialConditions {
  Eigen::Vector3d uav_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d uav_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d uav_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector2d pld_rel_pos{Eigen::Vector2d::Zero()};
  Eigen::Vector2d pld_rel_vel{Eigen::Vector2d::Zero()};
};

namespace details {
struct DisturbanceEstimate {
  Eigen::Vector3d gain;
  math::Integral<Eigen::Vector3d> integral;
  Eigen::Vector3d value;
};

}  // namespace details

}  // namespace control

#endif  // SLTS_CONTROL_DEFINITIONS_H_
