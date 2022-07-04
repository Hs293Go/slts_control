#ifndef INTEGRAL_H
#define INTEGRAL_H

#include <Eigen/Core>

namespace math {

namespace details {
template <typename Derived1, typename Derived2, typename Derived3>
auto Clamp(const Eigen::MatrixBase<Derived1>& x,
           const Eigen::MatrixBase<Derived2>& ub,
           const Eigen::MatrixBase<Derived3>& lb) {
  return x.cwiseMax(lb).cwiseMin(ub);
}

template <typename Derived>
void setEigenObj(Eigen::DenseBase<Derived>& dst, typename Derived::Scalar src) {
  dst.setConstant(src);
}

template <typename Derived1, typename Derived2>
void setEigenObj(Eigen::DenseBase<Derived2>& dst,
                 const Eigen::DenseBase<Derived1>& src) {
  dst = src;
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
  bool setBounds(T1&& ub, T2&& lb) {
    details::setEigenObj(ub_, std::forward<T1>(ub));
    details::setEigenObj(lb_, std::forward<T2>(lb));

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
#endif  // INTEGRAL_H
