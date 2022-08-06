// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "slts_control/robust_tracker.h"

#include <cstdint>
#include <limits>

#include "slts_control/eigen_utils.h"

namespace control {
RobustTracker::RobustTracker(double uav_mass, double pld_mass,
                             double cable_length)
    : kUavMass(uav_mass),
      kPldMass(pld_mass),
      kSysMass(kUavMass + kPldMass),
      kCableLen(cable_length),
      kCableLenSq(kCableLen * kCableLen),
      kPldWeight(kPldMass * 9.80665 * Eigen::Vector3d::UnitZ()),
      kUavWeight(kUavMass * 9.80665 * Eigen::Vector3d::UnitZ()),
      kSysWeight(kPldWeight + kUavWeight) {}

bool RobustTracker::computeControlOutput(std::uint64_t t) {
  if (!param_set_ || !ic_set_) {
    return false;
  }

  if (!computeFullVelocity()) {
    return false;
  }

  auto last_time = integrator_last_time_.load();
  auto dt = 1e-9 * (t - last_time);
  updateDisturbanceEstimates(dt);
  updateTranslationalErrors(dt);

  while (!integrator_last_time_.compare_exchange_weak(last_time, t)) {
  }

  auto sync_force =
      -kUavMass * (trans_cross_feeding_rates_ + translational_sync_);
  auto motion_compensator =
      -k_trim_ * (pld_abs_vel_ + trans_cross_feeding_ + raw_cross_feeding_);
  auto trans_compensator = -kPldMass * (trans_cross_feeding_rates_ +
                                        k_gen_trans_err_ * gen_trans_err_);

  auto trim_force = -kUavWeight + pld_trim_est_ - proj_de_;
  thrust_sp_ = sync_force + motion_compensator + trans_compensator + trim_force;
  return true;
}

bool RobustTracker::setParams(const Params& p) {
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

bool RobustTracker::setInitialConditions(std::uint64_t time,
                                         const InitialConditions& ic) {
  integrator_last_time_ = time;
  pld_speed_diff_time = time;

  uav_vel_ = ic.uav_vel;
  uav_acc_ = ic.uav_acc;

  setPayloadRelativePosVel(ic.pld_rel_pos, ic.pld_rel_vel);
  if (!computeFullVelocity()) {
    return ic_set_ = false;
  }
  return ic_set_ = true;
}

void RobustTracker::setPayloadTranslationalErrors(
    const Eigen::Vector3d& pld_pos_err, const Eigen::Vector3d& pld_pos_err_rates,
    const Eigen::Vector3d& pld_vel_err, const Eigen::Vector3d& pld_vel_sp) {
  pld_pos_err_ = pld_pos_err;
  pld_pos_err_rates_ = pld_pos_err_rates;
  pld_vel_err_ = pld_vel_err;
  pld_vel_sp_ = pld_vel_sp;
}

void RobustTracker::setUavAcceleration(const Eigen::Vector3d& uav_acc) {
  uav_acc_ = uav_acc;
}

void RobustTracker::setUavPosition(const Eigen::Vector3d& uav_pos) {
  uav_pos_ = uav_pos;
}

void RobustTracker::setUavVelocity(const Eigen::Vector3d& uav_vel) {
  uav_vel_ = uav_vel;
}

void RobustTracker::setActualThrust(const Eigen::Vector3d& thrust_act) {
  thrust_act_ = thrust_act;
}

void RobustTracker::setPayloadRelativePosition(
    std::uint64_t time, const Eigen::Vector2d& pld_rel_pos) {
  auto last_time = pld_speed_diff_time.load();
  const auto dt = 1e-9 * (time - last_time);

  while (!pld_speed_diff_time.compare_exchange_weak(last_time, time)) {
  }

  pld_rel_vel_ = (pld_rel_pos - pld_rel_pos_) / dt;
  pld_rel_pos_ = pld_rel_pos;
}

void RobustTracker::setPayloadRelativePosVel(
    const Eigen::Vector2d& pld_rel_pos, const Eigen::Vector2d& pld_rel_vel) {
  pld_rel_pos_ = pld_rel_pos;
  pld_rel_vel_ = pld_rel_vel;
}

bool RobustTracker::computeFullVelocity() {
  using std::sqrt;
  pld_rel_pos_full_.head<2>() = pld_rel_pos_;
  pld_rel_vel_full_.head<2>() = pld_rel_vel_;

  auto B_lc = Eigen::Matrix2d::Identity() -
              pld_rel_pos_ * pld_rel_pos_.transpose() / kCableLenSq;
  const double sq_nrm = pld_rel_pos_.squaredNorm();
  if (sq_nrm < kCableLenSq) {
    const double z_sq = kCableLenSq - sq_nrm;
    const double z = sqrt(z_sq);
    iz_ = 1.0 / z;

    pld_rel_pos_full_.z() = -z;
    pld_rel_vel_full_.z() = iz_ * pld_rel_pos_.dot(pld_rel_vel_);

    const Eigen::Vector2d B_rc = z / kCableLenSq * pld_rel_pos_;
    B_frak_ << B_lc, B_rc, B_rc.transpose(), sq_nrm / kCableLenSq;

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
      -k_filter_leak_ * filt_cross_feeding + k_filter_gain_ * raw_cross_feeding_;
  auto scaled_pos_err_rates = k_pos_err_ * pld_pos_err_rates_;
  trans_cross_feeding_rates_ = scaled_pos_err_rates + filt_cross_feeding_rates;

  filt_cross_feeding_.integrate(filt_cross_feeding_rates, dt);
}

void RobustTracker::updateDisturbanceEstimates(double dt) {
  uav_de_.value = uav_de_.integral.value();

  proj_de_ = uav_de_.value - uav_de_.value.dot(pld_rel_pos_full_) / kCableLenSq *
                                 pld_rel_pos_full_;
  // ∫ Σ(...) + Δ_T + (m_p + M_q) * g_I
  total_de_.value =
      total_de_.gain * (kSysMass * pld_abs_vel_ + kUavMass * pld_rel_vel_full_ -
                        total_de_.integral.value());

  computePayloadStateEstimates();

  uav_de_.integral.integrate(
      uav_de_.gain * B_frak_ *
          (kUavMass * uav_acc_ - thrust_act_ - kUavWeight - uav_de_.value),
      dt);

  total_de_.integral.integrate(
      total_de_.value + thrust_act_ + proj_de_ + kSysWeight, dt);
}

void RobustTracker::computePayloadStateEstimates() {
  using std::sqrt;
  pld_trim_est_ = -(kPldWeight + total_de_.value);
  const double sq_nrm = pld_trim_est_.squaredNorm();

  // r_d = l f_dxy / ||f_xdy||
  Eigen::Vector2d pld_rel_pos_sp;
  if (sq_nrm > 0.0) {
    pld_rel_pos_sp = kCableLen * pld_trim_est_.head<2>() / sqrt(sq_nrm);
  } else {
    pld_rel_pos_sp.setZero();
  }
  const Eigen::Vector2d swing_error = k_swing_ * (pld_rel_pos_ - pld_rel_pos_sp);
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