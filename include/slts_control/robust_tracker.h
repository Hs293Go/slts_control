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
#include "slts_control/eigen_utils.h"

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
    k_gen_trans_err_ = p.k_gen_trans_err;
    k_trim_ = p.k_trim;
    k_swing_ = p.k_swing;

    total_de_.scaling = p.total_de_scaling;
    utils::setEigenObj(p.total_de_lb, total_de_.lb);
    utils::setEigenObj(p.total_de_ub, total_de_.ub);

    proj_de_.scaling = p.proj_de_scaling;
    utils::setEigenObj(p.proj_de_lb, proj_de_.lb);
    utils::setEigenObj(p.proj_de_ub, proj_de_.ub);
    param_set_ = true;
    return true;
  }

  void updateDisturbanceEstimates(double dt);

  bool setPayloadRelativePosition(std::uint64_t time,
                                  const Eigen::Vector2d& pld_rel_pos);

  bool setPayloadRelativePosVel(const Eigen::Vector2d& pld_rel_pos,
                                const Eigen::Vector2d& pld_rel_vel);

  bool computeFullVelocity();
  void computePayloadStateEstimates();

  void computeControlOutput(std::uint64_t t);

  inline const Eigen::Vector3d& thrust_sp() const { return thrust_sp_; }
  inline double yaw_sp() const { return yaw_sp_; }

  Eigen::Vector3d uav_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d uav_acc{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond uav_att{Eigen::Quaterniond::Identity()};

 private:
  struct DisturbanceEstimate {
    double scaling;
    Eigen::Vector3d integral;
    Eigen::Vector3d ub;
    Eigen::Vector3d lb;
    Eigen::Vector3d value;
  };

  const double kUavMass;
  const double kPldMass;
  const double kSysMass;
  const double kCableLen;
  const double kCableLenSq;
  const Eigen::Vector3d kPldWeight;
  const Eigen::Vector3d kUavWeight;

  double k_gen_trans_err_;
  double k_trim_;
  double k_swing_;
  bool param_set_{false};

  Eigen::Vector2d pld_rel_pos_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d pld_rel_vel_{Eigen::Vector2d::Zero()};
  Eigen::Vector3d pld_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pld_abs_vel_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d thrust_;
  Eigen::Vector3d pld_rel_vel_full_;
  Eigen::Vector3d translational_sync_;
  Eigen::Vector3d gen_trans_err_;
  Eigen::Vector3d trans_cross_feeding_;
  Eigen::Vector3d trans_cross_feeding_rates_;
  Eigen::Vector3d augmented_swing_speed_;
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
