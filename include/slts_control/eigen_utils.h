// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <Eigen/Dense>

#define ZEROS(r, c) (Eigen::Matrix<double, r, c>::Zero())

#ifndef NDEBUG
#include <iostream>
#define DBG_MAT_FMT \
  (Eigen::IOFormat(Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"))

#define DBG_MAT_STREAM(stream, mat)                                      \
  do {                                                                   \
    stream << "Value of " #mat ":\n" << mat.format(DBG_MAT_FMT) << "\n"; \
  } while (0)

#define DBG_MAT(mat) DBG_MAT_STREAM(std::cout, mat);
#endif

namespace utils {

template <typename Derived1, typename Derived2, typename Derived3>
auto Clip(const Eigen::MatrixBase<Derived1>& x,
          const Eigen::MatrixBase<Derived2>& ub,
          const Eigen::MatrixBase<Derived3>& lb) {
  return x.cwiseMax(lb).cwiseMin(ub);
}

template <typename Derived, typename Scalar>
void setEigenObj(Scalar src, Eigen::DenseBase<Derived>& dst) {
  dst.setConstant(src);
}

template <typename Derived1, typename Derived2>
void setEigenObj(const Eigen::DenseBase<Derived1>& src,
                 Eigen::DenseBase<Derived2>& dst) {
  dst.setConstant(src);
}

}  // namespace utils

#endif  // EIGEN_UTILS_H
