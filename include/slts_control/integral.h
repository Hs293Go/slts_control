// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SLTS_CONTROL_INTEGRAL_H_
#define SLTS_CONTROL_INTEGRAL_H_

#include <Eigen/Core>
#include <utility>

namespace math {

namespace details {
template <typename Derived1, typename Derived2, typename Derived3>
auto Clamp(const Eigen::MatrixBase<Derived1>& x,
           const Eigen::MatrixBase<Derived2>& ub,
           const Eigen::MatrixBase<Derived3>& lb) {
  return x.cwiseMax(lb).cwiseMin(ub);
}

}  // namespace details

template <typename _MatrixType>
class Integral {
 public:
  using MatrixType = _MatrixType;
  enum {
    Size = MatrixType::RowsAtCompileTime,
    ColsAtCompileTime = MatrixType::ColsAtCompileTime,
    Options = MatrixType::Options,
    MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime
  };

  Integral() : value_(MatrixType::Zero()) {}

  template <typename Derived>
  explicit Integral(const Eigen::MatrixBase<Derived>& ic) : value_(ic) {}

  template <typename T1, typename T2>
  bool setBounds(const Eigen::MatrixBase<T1>& ub,
                 const Eigen::MatrixBase<T2>& lb) {
    ub_ = ub;
    lb_ = lb;

    if ((ub_.array() < ub_.array()).any()) {
      return bounds_set_ = false;
    }

    return bounds_set_ = true;
  }

  template <typename Derived, typename Scalar>
  void integrate(const Eigen::MatrixBase<Derived>& integrand, Scalar dt) {
    if (bounds_set_) {
      value_ = details::Clamp(value_ + integrand * dt, ub_, lb_);
    } else {
      value_ += integrand * dt;
    }
  }

  const MatrixType& value() const { return value_; }

 private:
  MatrixType value_;
  MatrixType ub_;
  MatrixType lb_;
  bool bounds_set_;
};
}  // namespace math
#endif  // SLTS_CONTROL_INTEGRAL_H_
