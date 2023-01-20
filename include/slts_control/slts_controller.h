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

class SLTSController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = SLTSControllerParams;
  using InitialConditions = SLTSControllerInitialConditions;
  using Status = SLTSControllerStatus;

  SLTSController();
  SLTSController(const SLTSController&) = delete;
  SLTSController(SLTSController&&) = delete;
  SLTSController& operator=(const SLTSController&) = delete;
  SLTSController& operator=(SLTSController&&) = delete;

  /**
   * @brief Loads controller parameters
   *
   * @param p A Params (SLTSControllerParams) struct containing the values for
   * the parameters
   * @return true All parameters are valid
   * @return false Some parameters are invalid, may be due to negative mass or
   * length parameters or inconsistent upper / lower bounds for bounded values
   */
  bool loadParams(const Params& p);

  /**
   * @brief Set the initial conditions for the controller
   *
   * @param ic A InitialConditions struct containing the initial values for the
   * states of the SLTS, including uav position, velocity, acceleration and
   * payload position and velocity w.r.t. the UAV
   * @return true All initial conditions are valid
   * @return false Initial conditions are invalid because supplied payload
   * relative position violated the cable length constraint
   */
  bool setInitialConditions(const InitialConditions& ic = {});

  /**
   * @brief Set the payload relative position (from exteroceptive measurement),
   * while using numerical differentiation to update payload relative velocity
   *
   * @tparam Mode Numerical differential mode. May be NumDiffMode::Forward or
   * NumDiffMode::Backward
   * @param dt Time difference for numerical differentiation
   * @param pld_rel_pos Payload relative position
   */
  template <typename Mode>
  void setPayloadRelativePosition(double dt, const Eigen::Vector2d& pld_rel_pos,
                                  Mode);

  /**
   * @brief Set the payload relative position and velocity. The latter is likely
   * obtained by Mocap (or optical flow?)
   *
   * @param pld_rel_pos Payload relative position
   * @param pld_rel_vel Payload relative velocity
   */
  void setPayloadRelativePosVel(const Eigen::Vector2d& pld_rel_pos,
                                const Eigen::Vector2d& pld_rel_vel);

  /**
   * @brief Set the payload translational errors
   *
   * @param pld_pos_err Payload position error, aka a measure of position error
   * that is normal to the desired path of the payload
   * @param pld_pos_err_rates Payload position error rates, time derivative of
   * the above
   * @param pld_vel_err Payload velocity error, aka a measure of position error
   * that is tangential to the desired path of the payload
   * @param pld_vel_sp Payload velocity setpoint
   */
  void setPayloadTranslationalErrors(const Eigen::Vector3d& pld_pos_err,
                                     const Eigen::Vector3d& pld_pos_err_rates,
                                     const Eigen::Vector3d& pld_vel_err,
                                     const Eigen::Vector3d& pld_vel_sp);

  /**
   * @brief Updates payload position and velocity absolute states (and a few
   * other auxiliary quantities) after payload relative position / velocity have
   * been set.
   *
   * @return true Payload relative position is valid
   * @return false Payload relative position violated cable length constraints
   */
  bool computeFullVelocity();

  /**
   * @brief Main control loop function. Runs one control loop and updates the
   * thrust setpoint
   *
   * @param dt Time difference for integration inside the control law
   * @return Status Status of the controller
   */
  Status computeControlOutput(double dt);

  Eigen::Vector3d& uav_pos() { return uav_pos_; }
  Eigen::Vector3d& uav_vel() { return uav_vel_; }
  Eigen::Vector3d& uav_acc() { return uav_acc_; }
  Eigen::Vector3d& thrust_act() { return thrust_act_; }

  const Eigen::Vector3d& pld_abs_pos() const { return pld_abs_pos_; }
  const Eigen::Vector2d& pld_rel_pos() const { return pld_rel_pos_; }
  const Eigen::Vector3d& pld_abs_vel() const { return pld_abs_vel_; }
  const Eigen::Vector2d& pld_rel_vel() const { return pld_rel_vel_; }

  const Eigen::Vector3d& thrust_sp() const { return thrust_sp_; }
  double yaw_sp() const { return yaw_sp_; }

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
  Frame frame_{Frame::kNED};

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
