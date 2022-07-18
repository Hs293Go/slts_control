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
#include <variant>

#include "slts_control/definitions.h"

namespace control {

class RobustTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = RobustTrackerParams;
  using InitialConditions = RobustTrackerInitialConditions;

  RobustTracker(double uav_mass, double pld_mass, double cable_length);

  RobustTracker() = delete;
  RobustTracker(const RobustTracker&) = delete;
  RobustTracker(RobustTracker&&) = delete;
  RobustTracker& operator=(const RobustTracker&) = delete;
  RobustTracker& operator=(RobustTracker&&) = delete;

  bool setParams(const Params& p);

  bool setInitialConditions(std::uint64_t time,
                            const InitialConditions& ic = {});

  void setUavVelocity(const Eigen::Vector3d& uav_vel);

  void setUavAcceleration(const Eigen::Vector3d& uav_acc);

  void setActualThrust(const Eigen::Vector3d& thrust_act);

  bool setPayloadRelativePosition(std::uint64_t time,
                                  const Eigen::Vector2d& pld_rel_pos);

  bool setPayloadRelativePosVel(const Eigen::Vector2d& pld_rel_pos,
                                const Eigen::Vector2d& pld_rel_vel);

  void setPayloadTranslationalErrors(const Eigen::Vector3d& pld_pos_err,
                                     const Eigen::Vector3d& pld_vel_err_rates,
                                     const Eigen::Vector3d& pld_vel_err,
                                     const Eigen::Vector3d& pld_vel_sp);

  bool computeControlOutput(std::uint64_t t);

  inline const Eigen::Vector3d& thrust_sp() const { return thrust_sp_; }
  inline double yaw_sp() const { return yaw_sp_; }

 private:
  using DisturbanceEstimate = details::DisturbanceEstimate;

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
  const Eigen::Vector3d kSysWeight;

  double k_pos_err_;
  double k_gen_trans_err_;
  double k_trim_;
  double k_swing_;
  double k_filter_gain_;
  double k_filter_leak_;
  bool param_set_{false};
  bool ic_set_{false};

  double iz_;

  Eigen::Vector3d uav_vel_;
  Eigen::Vector3d uav_acc_;
  Eigen::Vector2d pld_rel_pos_;
  Eigen::Vector2d pld_rel_vel_;
  Eigen::Vector3d pld_abs_vel_;

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

  std::mutex mtx_;

  std::atomic_uint64_t pld_speed_diff_time;
  std::atomic_uint64_t integrator_last_time_;
};

}  // namespace control

#endif  // ROBUST_TRACKER_H
