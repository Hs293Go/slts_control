// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <unordered_map>

#include "slts_control/robust_tracker.h"

using namespace std::string_literals;

#ifndef TEST_DATAFILE
#define TEST_DATAFILE \
  "YOU DID NOT SET THE TEST_DATAFILE MACRO CORRECTLY WHEN COMPILING THIS FILE!"
#endif

MATCHER_P(ContainsKey, key, key + (negation ? " not "s : " "s) + "found") {
  return arg.count(key) > 0;
}

class TestController : public ::testing::Test {
 public:
  static constexpr double kCableLength = 0.98;
  static constexpr double kUavMass = 1.63;
  static constexpr double kPldMass = 0.5;

  TestController() : tracker(kUavMass, kPldMass, kCableLength) {}

  std::unordered_map<std::string, Eigen::MatrixXd> dataset;
  control::RobustTracker tracker;

  void SetUp() {
    control::RobustTracker::Params p;
    ASSERT_TRUE(tracker.setParams(p));
    using nlohmann::json;

    json root;
    std::ifstream ifs(TEST_DATAFILE);
    ASSERT_TRUE(ifs.is_open()) << "Failed to open: " << TEST_DATAFILE << "\n";
    ASSERT_NO_THROW(ifs >> root) << "Json parsing failed!\n";

    for (const auto& tup : root.items()) {
      const auto& key = tup.key();
      const auto& it = tup.value();
      const auto& sz = it["size"];
      const auto& val = it["value"];
      dataset.emplace(key, Eigen::MatrixXd(int(sz[0]), int(sz[1])));
      std::copy(val.begin(), val.end(), dataset[key].data());
    }
  }
};

TEST_F(TestController, testDataFileKeys) {
  ASSERT_THAT(dataset, ContainsKey("uav_pos"));
  ASSERT_THAT(dataset, ContainsKey("uav_vel"));
  ASSERT_THAT(dataset, ContainsKey("uav_acc"));
  ASSERT_THAT(dataset, ContainsKey("pld_abs_pos"));
  ASSERT_THAT(dataset, ContainsKey("pld_rel_pos"));
  ASSERT_THAT(dataset, ContainsKey("pld_abs_vel"));
  ASSERT_THAT(dataset, ContainsKey("pld_rel_vel"));
  ASSERT_THAT(dataset, ContainsKey("thrust_cmd"));
  ASSERT_THAT(dataset, ContainsKey("thrust_act"));
  ASSERT_THAT(dataset, ContainsKey("pld_pos_err"));
  ASSERT_THAT(dataset, ContainsKey("pld_vel_err"));
  ASSERT_THAT(dataset, ContainsKey("pld_pos_err_rates"));
  ASSERT_THAT(dataset, ContainsKey("pld_vel_sp"));
  ASSERT_THAT(dataset, ContainsKey("proj_de"));
  ASSERT_THAT(dataset, ContainsKey("total_de"));
  ASSERT_THAT(dataset, ContainsKey("proj_de_est_err"));
  ASSERT_THAT(dataset, ContainsKey("total_de_est_err"));
  ASSERT_THAT(dataset, ContainsKey("pld_swing_est_err"));
}

TEST_F(TestController, testController) {
  int data_sz = dataset["tout"].size();

  for (int i = 0; i < data_sz - 1; ++i) {
    ASSERT_TRUE(tracker.setPayloadRelativePosVel(
        -dataset["pld_rel_pos"].col(i), -dataset["pld_rel_vel"].col(i)));
    tracker.setUavVelocity(dataset["uav_vel"].col(i));
    tracker.setUavAcceleration(dataset["uav_acc"].col(i));
    tracker.setActualThrust(dataset["thrust_act"].col(i));
    tracker.setPayloadTranslationalErrors(
        dataset["pld_pos_err"].col(i), dataset["pld_pos_err_rates"].col(i),
        dataset["pld_vel_err"].col(i), dataset["pld_vel_sp"].col(0));

    std::uint64_t ts = 1e9 * dataset["tout"].coeff(i + 1, 0);
    tracker.computeControlOutput(ts);

    const Eigen::Vector3d result = tracker.thrust_sp();
    const Eigen::Vector3d expected = dataset["thrust_cmd"].col(i);
    ASSERT_TRUE(result.isApprox(expected, 1e-10))
        << "Comparison failed on iteration: " << i << " where time is " << ts
        << " \n";
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}