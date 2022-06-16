// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <Eigen/Dense>

#define ZEROS(r, c) (Eigen::Matrix<double, r, c>::Zero())

namespace utils {

template <typename Derived1, typename Derived2, typename Derived3>
auto Clip(const Eigen::MatrixBase<Derived1>& x,
          const Eigen::MatrixBase<Derived2>& ub,
          const Eigen::MatrixBase<Derived3>& lb) {
  return x.cwiseMax(lb).cwiseMin(ub);
}
}  // namespace utils

#endif  // EIGEN_UTILS_H
