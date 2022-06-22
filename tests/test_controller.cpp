// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>

#include <Eigen/Core>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <unordered_map>

#include "slts_control/common.h"
#include "slts_control/robust_tracker.h"

using namespace std::string_literals;
MATCHER_P(ContainsKey, key, key + (negation ? " not "s : " "s) + "found") {
  return arg.count(key) > 0;
}

class TestController : public ::testing::Test {
 public:
  TestController() : tracker(Properties()) {
    readFromFile();

    struct Params {
      double k_pos_err = 0.24;
      double k_gen_trans_err = 0.10;
      double k_trim = 5;
      double k_swing = 0.15;
      double total_de_gain = 0.5;
      double total_de_ub = 10.0;
      double total_de_lb = -10.0;

      double uav_de_gain = 0.9;
      double uav_de_ub = 10.0;
      double uav_de_lb = -10.0;

      double k_filter_leak = 0.4;
      double k_filter_gain = 0.2;
      double filt_cross_feeding_lb = -1.0;
      double filt_cross_feeding_ub = 1.0;
    };

    tracker.setParams(Params{});
  }

  std::unordered_map<std::string, Eigen::MatrixXd> dataset;
  control::RobustTracker tracker;

 private:
  void readFromFile() {
    auto file = std::filesystem::path(__FILE__).replace_extension(".json");

    Json::Value root;
    std::ifstream ifs(file);
    ifs >> root;

    for (const auto& key : root.getMemberNames()) {
      const auto& it = root[key];
      const auto& sz = it["size"];
      const auto& val = it["value"];
      dataset.emplace(key, Eigen::MatrixXd(sz[0].asInt(), sz[1].asInt()));
      std::transform(val.begin(), val.end(), dataset[key].data(),
                     std::mem_fn(&Json::Value::asDouble));
    }
  }

  static common::SLTSProperty Properties() {
    common::SLTSProperty p;
    p.cable_length = 0.98;
    p.uav_mass = 1.63;
    p.pld_mass = 0.5;
    return p;
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