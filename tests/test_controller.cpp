// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>

#include <Eigen/Core>
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
  TestController() : tracker(Properties()) { readFromFile(); }

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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}