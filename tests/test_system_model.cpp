// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <istream>
#include <string>
#include <vector>

#include "slts_control/common.h"
#include "slts_control/system_model.h"

namespace fs = std::filesystem;
class TestSystemModel : public ::testing::Test {
 public:
  TestSystemModel() {
    slts_properties_.cable_length = 0.98;
    slts_properties_.uav_mass = 1.63;
    slts_properties_.pld_mass = 0.5;

    ics_.pld_abs_pos << 0.02, -0.02, 0.9795917517006766;
    ics_.pld_rel_pos << 0.02, -0.02;
    ics_.pld_abs_vel << 2.1, -1.0, 2.041666843894699e-03;
    ics_.pld_rel_vel << 0.1000, 0;
    read_data_ = readDataFile();
  }

  const common::SLTSProperty &slts_properties() { return slts_properties_; }
  const common::SLTSStates &ics() { return ics_; }
  bool read_data() const { return read_data_; }
  int len_dataset() const { return len_dataset_; }
  const Eigen::VectorXd &ts() { return ts_; }
  const Eigen::Matrix<double, 3, Eigen::Dynamic> &pld_abs_pos() {
    return pld_abs_pos_;
  }
  const Eigen::Matrix<double, 2, Eigen::Dynamic> &pld_rel_pos() {
    return pld_rel_pos_;
  }
  const Eigen::Matrix<double, 3, Eigen::Dynamic> &pld_abs_vel() {
    return pld_abs_vel_;
  }
  const Eigen::Matrix<double, 2, Eigen::Dynamic> &pld_rel_vel() {
    return pld_rel_vel_;
  }
  const Eigen::Matrix<double, 3, Eigen::Dynamic> &act_force() {
    return act_force_;
  }

  const Eigen::Vector3d &uav_dtb() { return uav_dtb_; }
  const Eigen::Vector3d &pld_dtb() { return pld_dtb_; }

 private:
  bool readDataFile() {
    const std::string file_location =
        fs::path(__FILE__).replace_extension(".bin");
    std::ifstream ifs(file_location);

    using BytesIter = std::istreambuf_iterator<char>;
    std::vector<char> buf(BytesIter{ifs}, BytesIter{});
    int sz = buf.size() * sizeof(char) / sizeof(double);
    if (sz % 13) {
      return false;
    }
    len_dataset_ = sz / 13;
    Eigen::Map<Eigen::MatrixXd> table(reinterpret_cast<double *>(buf.data()),
                                      len_dataset_, 14);
    pld_abs_pos_ = table.middleCols<3>(0).transpose();
    pld_rel_pos_ = table.middleCols<2>(3).transpose();
    pld_abs_vel_ = table.middleCols<3>(5).transpose();
    pld_rel_vel_ = table.middleCols<2>(8).transpose();
    act_force_ = table.middleCols<3>(10).transpose();
    return true;
  }
  common::SLTSProperty slts_properties_;
  common::SLTSStates ics_;

  bool read_data_;
  int len_dataset_;
  Eigen::VectorXd ts_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> pld_abs_pos_;
  Eigen::Matrix<double, 2, Eigen::Dynamic> pld_rel_pos_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> pld_abs_vel_;
  Eigen::Matrix<double, 2, Eigen::Dynamic> pld_rel_vel_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> act_force_;

  Eigen::Vector3d uav_dtb_{-1.0, 2.0, -0.3};
  Eigen::Vector3d pld_dtb_{-0.4, 0.8, -0.3};
};

TEST_F(TestSystemModel, matchOutputToFile) {
  constexpr double kDt = 0.01;
  ASSERT_TRUE(read_data());
  mdl::SLTSModel mdl(slts_properties(), ics());
  mdl::SLTSModel::StateType state;
  mdl::SLTSModel::InputType input;
  for (int i = 0; i < len_dataset() - 1; ++i) {
    input << act_force().col(i), pld_dtb(), uav_dtb();
    ASSERT_TRUE(mdl(kDt, input, state));
    std::cout << state.segment<3>(0) << " " << pld_abs_pos().col(i + 1) << "\n";
    ASSERT_TRUE(state.segment<3>(0).isApprox(pld_abs_pos().col(i + 1)));
  }
}
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}