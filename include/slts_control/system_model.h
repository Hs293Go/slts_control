// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SLUNG_LOAD_MODEL_H
#define SLUNG_LOAD_MODEL_H

#include <Eigen/Dense>

#include "slts_control/common.h"

namespace mdl {
class SLTSModel {
 public:
  using StateType = Eigen::Matrix<double, 10, 1>;
  using VelocityType = Eigen::Matrix<double, 5, 1>;
  using InputType = Eigen::Matrix<double, 9, 1>;

  SLTSModel(const common::SLTSProperty& slts_property,
            const common::SLTSStates& states);

  bool df(const InputType& u, StateType& dx);
  bool operator()(double dt, const InputType& u, StateType& dx);

 private:
  static const VelocityType kGrav;
  const double kUavMass;
  const double kPldMass;
  const double kSysMass;
  const double kCableLen;
  const double kCableLenSq;

  Eigen::Vector3d pld_abs_pos_;
  Eigen::Vector2d pld_rel_pos_;
  Eigen::Vector3d pld_abs_vel_;
  Eigen::Vector2d pld_rel_vel_;
  Eigen::Matrix<double, 5, 5> mass_;
};
}  // namespace mdl
#endif  // SLUNG_LOAD_MODEL_H
