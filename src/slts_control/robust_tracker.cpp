// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "slts_control/robust_tracker.h"

#include <cstdint>
#include <limits>

#include "slts_control/eigen_utils.h"

namespace control {
RobustTracker::RobustTracker(const common::SLTSProperty& slts_property)
    : kUavMass(slts_property.uav_mass),
      kPldMass(slts_property.pld_mass),
      kSysMass(kUavMass + kPldMass),
      kCableLen(slts_property.cable_length),
      kCableLenSq(kCableLen * kCableLen),
      kPldWeight(kPldMass * 9.8 * Eigen::Vector3d::UnitZ()),
      kUavWeight(kUavMass * 9.8 * Eigen::Vector3d::UnitZ()) {}

void RobustTracker::computeControlOutput(std::uint64_t t) {
  auto sync_force =
      -kUavMass * (trans_cross_feeding_rates_ + translational_sync_);
  auto motion_compensator =
      -k_trim_ * (uav_vel + trans_cross_feeding_ - raw_cross_feeding_);
  auto trans_compensator = -kPldMass * (trans_cross_feeding_rates_ +
                                        k_gen_trans_err_ * gen_trans_err_);

  auto trim_force = -kUavWeight + pld_trim_est_ - proj_de_.value;
  thrust_sp_ = sync_force + motion_compensator + trans_compensator + trim_force;
}

void RobustTracker::setPayloadTranslationalErrors(
    const Eigen::Vector3d& pld_pos_err, const Eigen::Vector3d& pld_vel_err,
    const Eigen::Vector3d& pld_vel_sp) {
  pld_pos_err_ = pld_pos_err;
  pld_vel_err_ = pld_vel_err;
  pld_vel_sp_ = pld_vel_sp;
}

bool RobustTracker::setPayloadRelativePosition(
    std::uint64_t time, const Eigen::Vector2d& pld_rel_pos) {
  auto last_time = pld_speed_diff_time.load();
  const auto dt = 1e-9 * (time - last_time);

  while (!pld_speed_diff_time.compare_exchange_weak(last_time, time)) {
  }

  pld_rel_vel_ = (pld_rel_pos - pld_rel_pos_) / dt;
  pld_rel_pos_ = pld_rel_pos;

  return computeFullVelocity();
}

bool RobustTracker::setPayloadRelativePosVel(
    const Eigen::Vector2d& pld_rel_pos, const Eigen::Vector2d& pld_rel_vel) {
  pld_rel_pos_ = pld_rel_pos;
  pld_rel_vel_ = pld_rel_vel;
  return computeFullVelocity();
}

bool RobustTracker::computeFullVelocity() {
  pld_rel_vel_full_.head<2>() = pld_rel_vel_;
  const Eigen::Vector2d swing_error = pld_rel_pos_ - pld_rel_pos_sp_est_;
  const Eigen::Vector2d gen_swing_speed = pld_rel_vel_ + swing_error;
  raw_cross_feeding_.head<2>() = gen_swing_speed;

  translational_sync_ = k_swing_ * pld_rel_vel_full_;

  auto B_lc = Eigen::Matrix2d::Identity() -
              pld_rel_pos_ * pld_rel_pos_.transpose() / kCableLenSq;
  const double sq_nrm = pld_rel_pos_.squaredNorm();
  if (sq_nrm < kCableLenSq) {
    const double iz_sq = 1.0 / (kCableLenSq - sq_nrm);
    const double iz = sqrt(iz_sq);

    // We compute only 3rd row of the B-matrix, since the 1-2 rows of the
    // B-matrix is identity
    const Eigen::Vector2d B3 = pld_rel_pos_ * iz;
    pld_rel_vel_full_.z() = B3.dot(pld_rel_vel_);
    raw_cross_feeding_.z() = B3.dot(gen_swing_speed);

    auto B_factor_rate =
        (iz_sq * pld_rel_pos_.dot(pld_rel_vel_) * pld_rel_pos_ + pld_rel_vel_) *
        iz;
    translational_sync_.z() += B_factor_rate.dot(swing_error);

    const Eigen::Vector2d B_rc =
        -sqrt(kCableLenSq - sq_nrm) / kCableLenSq * pld_rel_pos_;
    B_frak_ << B_lc, B_rc, B_rc.transpose(), sq_nrm / kCableLenSq;
    return true;
  }
  return false;
}

void RobustTracker::updateTranslationalErrors(double dt) {
  const Eigen::Vector3d scaled_pos_err = k_pos_err_ * pld_pos_err_;
  gen_trans_err_ = scaled_pos_err + pld_vel_err_;

  const Eigen::Vector3d& filt_cross_feeding = filt_cross_feeding_.value();
  trans_cross_feeding_ = scaled_pos_err + filt_cross_feeding - pld_vel_sp_;

  auto filt_cross_feeding_rates =
      -k_filter_leak_ * raw_cross_feeding_ + k_filter_gain_ * filt_cross_feeding;
  auto scaled_pos_err_rates = k_pos_err_ * pld_pos_err_rates_;
  trans_cross_feeding_rates_ = scaled_pos_err_rates + filt_cross_feeding_rates;

  filt_cross_feeding_.integrate(filt_cross_feeding_rates, dt);
}

void RobustTracker::updateDisturbanceEstimates(double dt) {
  auto thrust = uav_att.inverse() * Eigen::Vector3d::UnitZ() * thrust_sp_.norm();

  proj_de_.integral.integrate(
      B_frak_ * (kUavMass * uav_acc - thrust - kUavWeight - proj_de_.value), dt);
  proj_de_.value = proj_de_.scaling * proj_de_.integral.value();

  // ∫ Σ(...) + Δ_T + (m_p + M_q) * g_I
  total_de_.integral.integrate(total_de_.value + thrust_ + proj_de_.value, dt);
  total_de_.value = total_de_.scaling *
                    (kSysMass * pld_abs_vel_ + kUavMass * pld_rel_vel_full_ -
                     total_de_.integral.value());

  computePayloadStateEstimates();
}

void RobustTracker::computePayloadStateEstimates() {
  using std::sqrt;
  pld_trim_est_ = -(kPldWeight + total_de_.value);
  const double sq_nrm = pld_trim_est_.squaredNorm();
  if (sq_nrm > 0.0) {
    pld_rel_pos_sp_est_ = kCableLen * pld_trim_est_.head<2>() / sqrt(sq_nrm);
  } else {
    pld_rel_pos_sp_est_.setZero();
  }
}
}  // namespace control