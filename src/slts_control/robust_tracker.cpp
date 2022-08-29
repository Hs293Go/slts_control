// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "slts_control/robust_tracker.h"

#include <cstdint>
#include <limits>

#include "slts_control/definitions.h"
#include "slts_control/eigen_utils.h"

namespace control {
RobustTracker::RobustTracker() : param_set_(false), ic_set_(false) {}

bool RobustTracker::computeControlOutput(double dt) {
  if (!param_set_ || !ic_set_) {
    return false;
  }

  if (!computeFullVelocity()) {
    return false;
  }

  if (dt < 1e-10) {
    return false;
  }
  updateDisturbanceEstimates(dt);
  updateTranslationalErrors(dt);

  auto sync_force =
      -uav_mass_ * (trans_cross_feeding_rates_ + translational_sync_);
  auto motion_compensator =
      -k_trim_ * (pld_abs_vel_ + trans_cross_feeding_ + raw_cross_feeding_);
  auto trans_compensator = -pld_mass_ * (trans_cross_feeding_rates_ +
                                         k_gen_trans_err_ * gen_trans_err_);

  auto trim_force = -uav_weight_ + pld_trim_est_ - proj_de_;
  thrust_sp_ = sync_force + motion_compensator + trans_compensator + trim_force;
  return true;
}

bool RobustTracker::loadParams(const Params& p) {
  uav_mass_ = p.uav_mass;
  if (p.uav_mass < 0.0) {
    return false;
  }
  pld_mass_ = p.pld_mass;
  if (p.pld_mass < 0.0) {
    return false;
  }

  sys_mass_ = uav_mass_ + pld_mass_;

  cable_len_ = p.cable_length;
  if (cable_len_ < 0.0) {
    return false;
  }
  cable_len_sq_ = cable_len_ * cable_len_;

  const double grav_const = p.frame == Frame::kNED ? 9.80665 : -9.80665;
  pld_weight_ = pld_mass_ * grav_const * Eigen::Vector3d::UnitZ();
  uav_weight_ = uav_mass_ * grav_const * Eigen::Vector3d::UnitZ();
  sys_weight_ = pld_weight_ + uav_weight_;

  k_pos_err_ = p.k_pos_err;
  k_gen_trans_err_ = p.k_gen_trans_err;
  k_trim_ = p.k_trim;
  k_swing_ = p.k_swing;

  total_de_.gain = p.total_de_gain;
  if (p.total_de_ub.size() != 1 && p.total_de_ub.size() != 3) {
    return false;
  }
  if (p.total_de_lb.size() != 1 && p.total_de_lb.size() != 3) {
    return false;
  }
  if (!total_de_.integral.setBounds(p.total_de_ub, p.total_de_lb)) {
    return false;
  }

  uav_de_.gain = p.uav_de_gain;
  if (p.uav_de_ub.size() != 1 && p.uav_de_ub.size() != 3) {
    return false;
  }
  if (p.uav_de_lb.size() != 1 && p.uav_de_lb.size() != 3) {
    return false;
  }
  if (!uav_de_.integral.setBounds(p.uav_de_ub, p.uav_de_lb)) {
    return false;
  }

  k_filter_gain_ = p.k_filter_gain;
  k_filter_leak_ = p.k_filter_leak;
  if (p.filt_cross_feeding_ub.size() != 1 &&
      p.filt_cross_feeding_ub.size() != 3) {
    return false;
  }
  if (p.filt_cross_feeding_lb.size() != 1 &&
      p.filt_cross_feeding_lb.size() != 3) {
    return false;
  }
  if (!filt_cross_feeding_.setBounds(p.filt_cross_feeding_ub,
                                     p.filt_cross_feeding_lb)) {
    return false;
  }
  param_set_ = true;
  return true;
}

bool RobustTracker::setInitialConditions(const InitialConditions& ic) {
  uav_pos_ = ic.uav_pos;
  uav_vel_ = ic.uav_vel;
  uav_acc_ = ic.uav_acc;

  setPayloadRelativePosVel(ic.pld_rel_pos, ic.pld_rel_vel);
  if (!computeFullVelocity()) {
    return ic_set_ = false;
  }
  return ic_set_ = true;
}

void RobustTracker::setPayloadTranslationalErrors(
    const Eigen::Vector3d& pld_pos_err,
    const Eigen::Vector3d& pld_pos_err_rates,
    const Eigen::Vector3d& pld_vel_err, const Eigen::Vector3d& pld_vel_sp) {
  pld_pos_err_ = pld_pos_err;
  pld_pos_err_rates_ = pld_pos_err_rates;
  pld_vel_err_ = pld_vel_err;
  pld_vel_sp_ = pld_vel_sp;
}

template <>
void RobustTracker::setPayloadRelativePosition<NumDiffMode::Backward_t>(
    double dt, const Eigen::Vector2d& pld_rel_pos, NumDiffMode::Backward_t) {
  if (dt < 1e-10) {
    return;
  }
  pld_rel_vel_ = (pld_rel_pos - pld_rel_pos_) / dt;
  pld_rel_pos_ = pld_rel_pos;
}

template <>
void RobustTracker::setPayloadRelativePosition<NumDiffMode::Forward_t>(
    double dt, const Eigen::Vector2d& pld_rel_pos, NumDiffMode::Forward_t) {
  pld_rel_pos_ = numdiff_rel_pos_;
  if (dt < 1e-10) {
    return;
  }

  pld_rel_vel_ = (pld_rel_pos - numdiff_rel_pos_) / dt;
  numdiff_rel_pos_ = pld_rel_pos;
}

void RobustTracker::setPayloadRelativePosVel(
    const Eigen::Vector2d& pld_rel_pos, const Eigen::Vector2d& pld_rel_vel) {
  pld_rel_pos_ = pld_rel_pos;
  numdiff_rel_pos_ = pld_rel_pos;
  pld_rel_vel_ = pld_rel_vel;
}

bool RobustTracker::computeFullVelocity() {
  using std::sqrt;
  pld_rel_pos_full_.head<2>() = pld_rel_pos_;
  pld_rel_vel_full_.head<2>() = pld_rel_vel_;

  auto B_lc = Eigen::Matrix2d::Identity() -
              pld_rel_pos_ * pld_rel_pos_.transpose() / cable_len_sq_;
  const double sq_nrm = pld_rel_pos_.squaredNorm();
  if (sq_nrm < cable_len_sq_) {
    const double z_sq = cable_len_sq_ - sq_nrm;
    const double z = sqrt(z_sq);
    iz_ = 1.0 / z;

    pld_rel_pos_full_.z() = -z;
    pld_rel_vel_full_.z() = iz_ * pld_rel_pos_.dot(pld_rel_vel_);

    const Eigen::Vector2d B_rc = z / cable_len_sq_ * pld_rel_pos_;
    B_frak_ << B_lc, B_rc, B_rc.transpose(), sq_nrm / cable_len_sq_;

    pld_abs_pos_ = uav_pos_ - pld_rel_pos_full_;
    pld_abs_vel_ = uav_vel_ - pld_rel_vel_full_;
    return true;
  }
  iz_ = -1.0;
  return false;
}

void RobustTracker::updateTranslationalErrors(double dt) {
  const Eigen::Vector3d scaled_pos_err = k_pos_err_ * pld_pos_err_;
  gen_trans_err_ = scaled_pos_err + pld_vel_err_;

  const Eigen::Vector3d& filt_cross_feeding = filt_cross_feeding_.value();
  trans_cross_feeding_ = scaled_pos_err + filt_cross_feeding - pld_vel_sp_;

  const Eigen::Vector3d filt_cross_feeding_rates =
      -k_filter_leak_ * filt_cross_feeding +
      k_filter_gain_ * raw_cross_feeding_;
  auto scaled_pos_err_rates = k_pos_err_ * pld_pos_err_rates_;
  trans_cross_feeding_rates_ = scaled_pos_err_rates + filt_cross_feeding_rates;

  filt_cross_feeding_.integrate(filt_cross_feeding_rates, dt);
}

void RobustTracker::updateDisturbanceEstimates(double dt) {
  uav_de_.value = uav_de_.integral.value();

  proj_de_ = uav_de_.value - uav_de_.value.dot(pld_rel_pos_full_) /
                                 cable_len_sq_ * pld_rel_pos_full_;
  // ∫ Σ(...) + Δ_T + (m_p + M_q) * g_I
  total_de_.value = total_de_.gain *
                    (sys_mass_ * pld_abs_vel_ + uav_mass_ * pld_rel_vel_full_ -
                     total_de_.integral.value());

  computePayloadStateEstimates();

  uav_de_.integral.integrate(
      uav_de_.gain * B_frak_ *
          (uav_mass_ * uav_acc_ - thrust_act_ - uav_weight_ - uav_de_.value),
      dt);

  total_de_.integral.integrate(
      total_de_.value + thrust_act_ + proj_de_ + sys_weight_, dt);
}

void RobustTracker::computePayloadStateEstimates() {
  using std::sqrt;
  pld_trim_est_ = -(pld_weight_ + total_de_.value);
  const double sq_nrm = pld_trim_est_.squaredNorm();

  // r_d = l f_dxy / ||f_xdy||
  Eigen::Vector2d pld_rel_pos_sp;
  if (sq_nrm > 0.0) {
    pld_rel_pos_sp = cable_len_ * pld_trim_est_.head<2>() / sqrt(sq_nrm);
  } else {
    pld_rel_pos_sp.setZero();
  }
  const Eigen::Vector2d swing_error =
      k_swing_ * (pld_rel_pos_ - pld_rel_pos_sp);
  const Eigen::Vector2d gen_swing_speed = pld_rel_vel_ + swing_error;
  raw_cross_feeding_.head<2>() = gen_swing_speed;

  if (iz_ > 0.0) {
    const double iz_sq = iz_ * iz_;
    raw_cross_feeding_.z() = iz_ * pld_rel_pos_.dot(gen_swing_speed);

    auto B_factor_rate =
        (iz_sq * pld_rel_pos_.dot(pld_rel_vel_) * pld_rel_pos_ + pld_rel_vel_) *
        iz_;
    translational_sync_ = k_swing_ * pld_rel_vel_full_;
    translational_sync_.z() += B_factor_rate.dot(swing_error);
  }
}
}  // namespace control
