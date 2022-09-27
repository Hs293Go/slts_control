// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SLTS_CONTROL_ROBUST_TRACKER_H_
#define SLTS_CONTROL_ROBUST_TRACKER_H_

#include <Eigen/Dense>
#include <cstdint>

#include "slts_control/definitions.h"

namespace control {

class RobustTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = RobustTrackerParams;
  using InitialConditions = RobustTrackerInitialConditions;
  using Status = RobustTrackerStatus;

  RobustTracker();
  RobustTracker(const RobustTracker&) = delete;
  RobustTracker(RobustTracker&&) = delete;
  RobustTracker& operator=(const RobustTracker&) = delete;
  RobustTracker& operator=(RobustTracker&&) = delete;

  bool loadParams(const Params& p);

  bool setInitialConditions(const InitialConditions& ic = {});

  template <typename Mode>
  void setPayloadRelativePosition(double dt, const Eigen::Vector2d& pld_rel_pos,
                                  Mode);

  void setPayloadRelativePosVel(const Eigen::Vector2d& pld_rel_pos,
                                const Eigen::Vector2d& pld_rel_vel);

  void setPayloadTranslationalErrors(const Eigen::Vector3d& pld_pos_err,
                                     const Eigen::Vector3d& pld_pos_err_rates,
                                     const Eigen::Vector3d& pld_vel_err,
                                     const Eigen::Vector3d& pld_vel_sp);

  bool computeFullVelocity();
  Status computeControlOutput(double dt);

  inline Eigen::Vector3d& uav_pos() { return uav_pos_; }
  inline Eigen::Vector3d& uav_vel() { return uav_vel_; }
  inline Eigen::Vector3d& uav_acc() { return uav_acc_; }
  inline Eigen::Vector3d& thrust_act() { return thrust_act_; }

  inline const Eigen::Vector3d& pld_abs_pos() const { return pld_abs_pos_; }
  inline const Eigen::Vector3d& pld_abs_vel() const { return pld_abs_vel_; }

  inline const Eigen::Vector3d& thrust_sp() const { return thrust_sp_; }
  inline double yaw_sp() const { return yaw_sp_; }

 private:
  using DisturbanceEstimate = details::DisturbanceEstimate;

  void computePayloadStateEstimates();

  void updateTranslationalErrors(double dt);

  void updateDisturbanceEstimates(double dt);

  double uav_mass_;
  double pld_mass_;
  double sys_mass_;
  double cable_len_;
  double cable_len_sq_;
  Eigen::Vector3d pld_weight_;
  Eigen::Vector3d uav_weight_;
  Eigen::Vector3d sys_weight_;

  Eigen::Vector3d k_pos_err_;
  Eigen::Vector3d k_gen_trans_err_;
  double k_trim_;
  double k_swing_;
  double k_filter_gain_;
  double k_filter_leak_;
  bool param_set_{false};
  bool ic_set_{false};

  double iz_;

  Eigen::Vector3d uav_pos_;
  Eigen::Vector3d uav_vel_;
  Eigen::Vector3d uav_acc_;
  Eigen::Vector2d pld_rel_pos_;
  Eigen::Vector2d pld_rel_vel_;
  Eigen::Vector3d pld_abs_pos_;
  Eigen::Vector3d pld_abs_vel_;

  Eigen::Vector2d numdiff_rel_pos_;

  math::Integral<Eigen::Vector3d> filt_cross_feeding_;
  Eigen::Vector3d thrust_;
  Eigen::Vector3d pld_rel_pos_full_;
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

  DisturbanceEstimate uav_de_;

  // d⊥
  Eigen::Vector3d proj_de_;

  Eigen::Vector3d pld_trim_est_;
  Eigen::Vector3d thrust_sp_;
  Eigen::Vector3d thrust_act_;
  double yaw_sp_;
};

}  // namespace control

#endif  // SLTS_CONTROL_ROBUST_TRACKER_H_
