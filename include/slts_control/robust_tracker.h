// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef ROBUST_TRACKER_H
#define ROBUST_TRACKER_H

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>
#include <mutex>

#include "slts_control/common.h"
#include "slts_control/integral.h"

namespace control {

class RobustTracker {
 public:
  RobustTracker(const common::SLTSProperty& slts_property);

  RobustTracker() = delete;
  RobustTracker(const RobustTracker&) = delete;
  RobustTracker(RobustTracker&&) = delete;
  RobustTracker& operator=(const RobustTracker&) = delete;
  RobustTracker& operator=(RobustTracker&&) = delete;

  template <typename T>
  bool setParams(const T& p) {
    k_pos_err_ = p.k_pos_err;
    k_gen_trans_err_ = p.k_gen_trans_err;
    k_trim_ = p.k_trim;
    k_swing_ = p.k_swing;

    total_de_.scaling = p.total_de_scaling;
    if (!total_de_.integral.setBounds(p.total_de_ub, p.total_de_lb)) {
      return false;
    }

    proj_de_.scaling = p.proj_de_scaling;
    if (!proj_de_.integral.setBounds(p.proj_de_ub, p.proj_de_lb)) {
      return false;
    }

    if (!filt_cross_feeding_.setBounds(p.filt_cross_feeding_ub,
                                       p.filt_cross_feeding_lb)) {
      return false;
    }
    param_set_ = true;
    return true;
  }

  bool setPayloadRelativePosition(std::uint64_t time,
                                  const Eigen::Vector2d& pld_rel_pos);

  bool setPayloadRelativePosVel(const Eigen::Vector2d& pld_rel_pos,
                                const Eigen::Vector2d& pld_rel_vel);

  void computeControlOutput(std::uint64_t t);

  inline const Eigen::Vector3d& thrust_sp() const { return thrust_sp_; }
  inline double yaw_sp() const { return yaw_sp_; }

  Eigen::Vector3d uav_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d uav_acc{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond uav_att{Eigen::Quaterniond::Identity()};

 private:
  struct DisturbanceEstimate {
    double scaling;
    math::Integral<Eigen::Vector3d> integral;
    Eigen::Vector3d value;
  };

  bool computeFullVelocity();

  void computePayloadStateEstimates();

  void updateTranslationalErrors(double dt);

  void updateDisturbanceEstimates(double dt);

  const double kUavMass;
  const double kPldMass;
  const double kSysMass;
  const double kCableLen;
  const double kCableLenSq;
  const Eigen::Vector3d kPldWeight;
  const Eigen::Vector3d kUavWeight;

  double k_pos_err_;
  double k_gen_trans_err_;
  double k_trim_;
  double k_swing_;
  double k_filter_gain_;
  double k_filter_leak_;
  bool param_set_{false};

  Eigen::Vector2d pld_rel_pos_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d pld_rel_vel_{Eigen::Vector2d::Zero()};
  Eigen::Vector3d pld_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pld_abs_vel_{Eigen::Vector3d::Zero()};

  math::Integral<Eigen::Vector3d> filt_cross_feeding_;
  Eigen::Vector3d thrust_;
  Eigen::Vector3d pld_rel_vel_full_;
  Eigen::Vector3d translational_sync_;
  Eigen::Vector3d pld_pos_err_;
  Eigen::Vector3d pld_pos_err_rates_;
  Eigen::Vector3d pld_vel_err_;
  Eigen::Vector3d pld_vel_sp_;
  Eigen::Vector3d gen_trans_err_;
  Eigen::Vector3d trans_cross_feeding_;
  Eigen::Vector3d trans_cross_feeding_rates_;
  Eigen::Vector3d raw_cross_feeding_;
  Eigen::Matrix3d B_frak_;

  // dT
  DisturbanceEstimate total_de_;

  // d⊥
  DisturbanceEstimate proj_de_;

  // r_d = l f_dxy / ||f_xdy||
  Eigen::Vector2d pld_rel_pos_sp_est_;
  Eigen::Vector3d pld_trim_est_;

  Eigen::Vector3d thrust_sp_;
  double yaw_sp_;

  std::mutex mtx_;

  std::atomic_uint64_t pld_speed_diff_time;
};

}  // namespace control

#endif  // ROBUST_TRACKER_H
