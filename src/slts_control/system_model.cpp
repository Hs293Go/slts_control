// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "slts_control/system_model.h"

namespace mdl {

const SLTSModel::VelocityType SLTSModel::kGrav =
    (SLTSModel::VelocityType() << Eigen::Vector3d::UnitZ() * 9.80665,
     Eigen::Vector2d::Zero())
        .finished();

SLTSModel::SLTSModel(const common::SLTSProperty& slts_property,
                     const common::SLTSStates& states)
    : kUavMass(slts_property.uav_mass),
      kPldMass(slts_property.pld_mass),
      kSysMass(kUavMass + kPldMass),
      kCableLen(slts_property.cable_length),
      kCableLenSq(kCableLen * kCableLen),
      pld_abs_pos_(states.pld_abs_pos),
      pld_rel_pos_(states.pld_rel_pos),
      pld_abs_vel_(states.pld_abs_vel),
      pld_rel_vel_(states.pld_rel_vel) {
  mass_.topLeftCorner<3, 3>() = kSysMass * Eigen::Matrix3d::Identity();
}

bool SLTSModel::df(const InputType& u, StateType& dx) {
  const double sq_nrm = pld_rel_pos_.squaredNorm();
  if (sq_nrm > kCableLenSq) {
    return false;
  }
  VelocityType force;
  VelocityType disturbance;
  VelocityType coriolis;
  const double t1 = pld_rel_pos_.dot(pld_rel_vel_);
  const double t2 = pld_rel_vel_.squaredNorm();
  const double t3 = 1.0 / (kCableLenSq - sq_nrm);
  const double t4 = sqrt(t3);
  const double t5 = t1 * t4;
  const double t6 = t2 + t5 * t5;

  coriolis << Eigen::Vector2d::Zero(),  //
      kUavMass * t4 * t6,               //
      kUavMass * t3 * t6 * pld_rel_pos_;

  const Eigen::Vector3d thst = u.head<3>();
  Eigen::Ref<const Eigen::Vector3d> pld_dist(u.segment<3>(3));
  Eigen::Ref<const Eigen::Vector3d> uav_dist(u.segment<3>(6));
  force << thst, thst.head<2>() + t4 * thst.z() * pld_rel_pos_;

  disturbance << pld_dist + uav_dist, uav_dist.head<2>(),
      t4 * uav_dist.z() * pld_rel_pos_;
  Eigen::Matrix<double, 3, 2> B;
  B << Eigen::Matrix2d::Identity(), pld_rel_pos_.transpose() * t4;
  mass_.topRightCorner<3, 2>() = kUavMass * B;
  mass_.bottomLeftCorner<2, 3>() = kUavMass * B.transpose();
  mass_.bottomRightCorner<2, 2>() =
      kUavMass * t3 * pld_rel_pos_ * pld_rel_pos_.transpose();

  dx.head<5>() << pld_abs_vel_, pld_rel_vel_;
  dx.tail<5>() = mass_.ldlt().solve(-coriolis + force + disturbance) + kGrav;
  return true;
}

bool SLTSModel::operator()(double dt, const InputType& u, StateType& x) {
  StateType dx;
  if (!df(u, dx)) {
    return false;
  }
  x << pld_abs_pos_, pld_rel_pos_, pld_abs_vel_, pld_rel_vel_;
  x += dt * dx;
  pld_abs_pos_ = x.segment<3>(0);
  pld_rel_pos_ = x.segment<2>(3);
  pld_abs_vel_ = x.segment<3>(5);
  pld_rel_vel_ = x.segment<2>(8);
  return true;
}
}  // namespace mdl