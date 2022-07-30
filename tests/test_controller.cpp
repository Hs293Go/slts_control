// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unordered_map>

#include "slts_control/robust_tracker.h"

using namespace std::string_literals;

#ifndef TEST_DATAFILE
#error YOU DID NOT SET THE TEST DATAFILE MACRO CORRECTLY WHEN COMPILING THIS FILE!
#endif

#ifndef TEST_PARAMFILE
#error YOU DID NOT SET THE TEST DATAFILE MACRO CORRECTLY WHEN COMPILING THIS FILE!
#endif

class TestController : public ::testing::Test {
 public:
  static constexpr double kCableLength = 0.98;
  static constexpr double kUavMass = 1.63;
  static constexpr double kPldMass = 0.5;

  TestController() : tracker(kUavMass, kPldMass, kCableLength) {}

  std::unordered_map<std::string, Eigen::MatrixXd> dataset;
  control::RobustTracker tracker;

  void SetUp() {
    using nlohmann::json;
    control::RobustTracker::Params p;
    json root;
    {
      std::ifstream ifs(TEST_PARAMFILE);
      ASSERT_TRUE(ifs.is_open()) << "Failed to open: " << TEST_DATAFILE << "\n";
      ASSERT_NO_THROW(ifs >> root) << "Json parsing failed!\n";
    }
    ASSERT_NO_THROW(p.k_swing = root["control"]["k_swing"]);
    ASSERT_NO_THROW(p.k_trim = root["control"]["k_trim"]);
    ASSERT_NO_THROW(p.k_filter_leak = root["control"]["k_filter_leak"]);
    ASSERT_NO_THROW(p.k_filter_gain = root["control"]["k_filter_gain"]);
    ASSERT_NO_THROW(p.k_gen_trans_err = root["control"]["k_gen_trans_err"]);
    ASSERT_NO_THROW(p.k_pos_err = root["control"]["k_pos_err"]);
    ASSERT_NO_THROW(p.total_de_gain = root["control"]["total_de_gain"]);
    ASSERT_NO_THROW(p.uav_de_gain = root["control"]["uav_de_gain"]);
    ASSERT_TRUE(tracker.setParams(p));

    {
      std::ifstream ifs(TEST_DATAFILE);
      ASSERT_TRUE(ifs.is_open()) << "Failed to open: " << TEST_DATAFILE << "\n";
      ASSERT_NO_THROW(ifs >> root) << "Json parsing failed!\n";
    }

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

TEST_F(TestController, testController) {
  Eigen::Vector2d pld_rel_pos;
  Eigen::Vector2d pld_rel_vel;
  Eigen::Vector3d uav_vel;
  Eigen::Vector3d uav_acc;
  Eigen::Vector3d thrust_act;
  Eigen::Vector3d pld_pos_err;
  Eigen::Vector3d pld_pos_err_rates;
  Eigen::Vector3d pld_vel_err;
  Eigen::Vector3d pld_vel_sp;
  Eigen::VectorXd tout;
  ASSERT_NO_THROW(tout = dataset["tout"]);
  tracker.setInitialConditions(tout[0]);
  int data_sz = dataset["tout"].size();
  ASSERT_GT(data_sz, 0);
  for (int i = 0; i < data_sz - 1; ++i) {
    ASSERT_NO_THROW(pld_rel_pos = -dataset["pld_rel_pos"].col(i));
    ASSERT_NO_THROW(pld_rel_vel = -dataset["pld_rel_vel"].col(i));
    ASSERT_NO_THROW(uav_vel = dataset["uav_vel"].col(i));
    ASSERT_NO_THROW(uav_acc = dataset["uav_acc"].col(i));
    ASSERT_NO_THROW(thrust_act = dataset["thrust_act"].col(i));
    ASSERT_NO_THROW(pld_pos_err_rates = dataset["pld_pos_err_rates"].col(i));
    ASSERT_NO_THROW(pld_pos_err = dataset["pld_pos_err"].col(i));
    ASSERT_NO_THROW(pld_vel_err = dataset["pld_vel_err"].col(i));
    ASSERT_NO_THROW(pld_vel_sp = dataset["pld_vel_sp"].col(i));
    ASSERT_TRUE(tracker.setPayloadRelativePosVel(pld_rel_pos, pld_rel_vel));
    tracker.setUavVelocity(uav_vel);
    tracker.setUavAcceleration(uav_acc);
    tracker.setActualThrust(thrust_act);
    tracker.setPayloadTranslationalErrors(pld_pos_err, pld_pos_err_rates,
                                          pld_vel_err, pld_vel_sp);

    std::uint64_t ts = 1e9 * tout[i + 1];
    tracker.computeControlOutput(ts);

    const Eigen::Vector3d result = tracker.thrust_sp();
    const Eigen::Vector3d expected = dataset["thrust_cmd"].col(i);
    ASSERT_TRUE(result.isApprox(expected, 1e-8))
        << "Comparison failed on iteration: " << i << " where time is " << ts
        << " \n";
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}