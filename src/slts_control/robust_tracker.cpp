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

void RobustTracker::computeControlOutput(std::uint64_t t) {}

void RobustTracker::setPayloadRelativePosition(
    std::uint64_t time, const Eigen::Vector2d& pld_rel_pos) {
  auto last_time = pld_speed_diff_time.load();
  const auto dt = 1e-9 * (time - last_time);

  while (!pld_speed_diff_time.compare_exchange_weak(last_time, time)) {
  }

  pld_rel_vel_ = (pld_rel_pos - pld_rel_pos_) / dt;
  pld_rel_pos_ = pld_rel_pos;
}

void RobustTracker::computeFullVelocity() {
  pld_rel_vel_full_.head<2>() = pld_rel_vel_;
  const Eigen::Vector2d gen_swing_speed = pld_rel_vel_ + swing_error_;
  augmented_swing_speed_.head<2>() = gen_swing_speed;

  translational_sync_.head<2>() = k_swing * pld_rel_vel_;

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
    augmented_swing_speed_.z() = B3.dot(gen_swing_speed);

    auto B_factor_rate =
        (iz_sq * pld_rel_pos_.dot(pld_rel_vel_) * pld_rel_pos_ + pld_rel_vel_) *
        iz;
    translational_sync_.z() =
        augmented_swing_speed_.z() + B_factor_rate.dot(swing_error_);

    const Eigen::Vector2d B_rc =
        -sqrt(kCableLenSq - sq_nrm) / kCableLenSq * pld_rel_pos_;
    B_frak_ << B_lc, B_rc, B_rc.transpose(), sq_nrm / kCableLenSq;

  } else {
    pld_rel_vel_full_.z() = 0.0;
    augmented_swing_speed_.z() = 0.0;
    translational_sync_.z() = 0.0;

    B_frak_ << B_lc, ZEROS(2, 1), ZEROS(1, 2), sq_nrm / kCableLenSq;
  }
}

void RobustTracker::computeSyncForce() {
  sync_force_ = -kUavMass * (trans_cross_feeding_rates_ + translational_sync_);
}

void RobustTracker::computeMotionCompensator() {
  motion_compensator_ =
      -k_trim * (uav_vel + trans_cross_feeding_ - augmented_swing_speed_);
}

void RobustTracker::computeTranslationalCompensator() {
  trans_compensator_ = -kPldMass * (trans_cross_feeding_rates_ +
                                    k_gen_trans_err * gen_trans_err_);
}

void RobustTracker::updateProjectedDE(double dt) {
  auto thrust = uav_att.inverse() * Eigen::Vector3d::UnitZ() * thrust_sp_.norm();

  auto integrand =
      B_frak_ * (kUavMass * uav_acc - thrust - kUavWeight - proj_de_.value);
  proj_de_.integral =
      utils::Clip(proj_de_.integral + integrand * dt, proj_de_.ub, proj_de_.lb);
  proj_de_.value = proj_de_.scaling * proj_de_.integral;
}

void RobustTracker::updateTotalDE(double dt) {
  // ∫ Σ(...) + Δ_T + (m_p + M_q) * g_I
  const Eigen::Vector3d integrand = total_de_.value + thrust_ + proj_de_.value;
  total_de_.integral += integrand * dt;
  total_de_.value =
      total_de_.scaling * (kSysMass * pld_abs_vel +
                           kUavMass * pld_rel_vel_full_ - total_de_.integral);
}

void RobustTracker::computePayloadEquilibrium() {
  pld_trim_ = -(kPldWeight + total_de_.value);
  trim_force_ = -kUavWeight + pld_trim_ - proj_de_.value;

  const double sq_nrm = pld_trim_.squaredNorm();
  Eigen::Vector2d pld_rel_pos_sp;
  if (sq_nrm > 0.0) {
    pld_rel_pos_sp = kCableLen * pld_trim_.head<2>() / sqrt(sq_nrm);
  } else {
    pld_rel_pos_sp.setZero();
  }
  swing_error_ = k_swing * (pld_rel_pos_ - pld_rel_pos_sp);
}
}  // namespace control