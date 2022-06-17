// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <unordered_map>

#include "slts_control/common.h"
#include "slts_control/robust_tracker.h"

class TestController : public ::testing::Test {
 public:
  TestController() : tracker(Properties()) {}

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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}