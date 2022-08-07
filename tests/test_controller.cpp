// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unordered_map>

#include "slts_control/definitions.h"
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

TEST_F(TestController, testPayloadAbsoluteStatesComputation) {
  Eigen::Vector2d pld_rel_pos;
  Eigen::Vector2d pld_rel_vel;
  Eigen::Vector3d pld_abs_pos;
  Eigen::Vector3d uav_pos;
  Eigen::Vector3d uav_vel;
  Eigen::VectorXd tout;
  Eigen::Vector3d result;
  Eigen::Vector3d expected;
  ASSERT_NO_THROW(tout = dataset.at("tout"));
  tracker.setInitialConditions();
  int data_sz = dataset.at("tout").size();
  ASSERT_GT(data_sz, 0);
  for (int i = 0; i < data_sz - 1; ++i) {
    ASSERT_NO_THROW(uav_pos = dataset.at("uav_pos").col(i));
    tracker.uav_pos() = uav_pos;

    ASSERT_NO_THROW(uav_vel = dataset.at("uav_vel").col(i));
    tracker.uav_vel() = uav_vel;

    ASSERT_NO_THROW(pld_rel_pos = -dataset.at("pld_rel_pos").col(i));
    ASSERT_NO_THROW(pld_rel_vel = -dataset.at("pld_rel_vel").col(i));
    tracker.setPayloadRelativePosVel(pld_rel_pos, pld_rel_vel);

    tracker.computeFullVelocity();
    result = tracker.pld_abs_pos();
    ASSERT_NO_THROW(expected = dataset.at("pld_abs_pos").col(i));
    ASSERT_TRUE(result.isApprox(expected, 1e-8))
        << "Comparison failed on iteration: " << i << " \n";
    result = tracker.pld_abs_vel();
    ASSERT_NO_THROW(expected = dataset.at("pld_abs_vel").col(i));
    ASSERT_TRUE(result.isApprox(expected, 1e-8))
        << "Comparison failed on iteration: " << i << " \n";
  }
}

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
  ASSERT_NO_THROW(tout = dataset.at("tout"));
  control::RobustTracker::InitialConditions ic;
  ASSERT_NO_THROW(ic.pld_rel_pos = -dataset.at("pld_rel_pos").col(0));
  ASSERT_NO_THROW(ic.pld_rel_vel = -dataset.at("pld_rel_vel").col(0));
  ASSERT_NO_THROW(ic.uav_acc = dataset.at("uav_acc").col(0));
  ASSERT_NO_THROW(ic.uav_vel = dataset.at("uav_vel").col(0));
  ASSERT_TRUE(tracker.setInitialConditions(ic));
  int data_sz = dataset.at("tout").size();
  ASSERT_GT(data_sz, 0);
  for (int i = 0; i < data_sz - 1; ++i) {
    double dt = tout[i + 1] - tout[i];
    ASSERT_NO_THROW(pld_rel_pos = -dataset.at("pld_rel_pos").col(i));
    ASSERT_NO_THROW(uav_vel = dataset.at("uav_vel").col(i));
    ASSERT_NO_THROW(uav_acc = dataset.at("uav_acc").col(i));
    ASSERT_NO_THROW(thrust_act = dataset.at("thrust_act").col(i));
    ASSERT_NO_THROW(pld_pos_err_rates = dataset.at("pld_pos_err_rates").col(i));
    ASSERT_NO_THROW(pld_pos_err = dataset.at("pld_pos_err").col(i));
    ASSERT_NO_THROW(pld_vel_err = dataset.at("pld_vel_err").col(i));
    ASSERT_NO_THROW(pld_vel_sp = dataset.at("pld_vel_sp").col(i));
    tracker.setPayloadRelativePosition(dt, -dataset.at("pld_rel_pos").col(i + 1),
                                       control::NumDiffMode::Forward);
    tracker.uav_vel() = uav_vel;
    tracker.uav_acc() = uav_acc;
    tracker.thrust_act() = thrust_act;
    tracker.setPayloadTranslationalErrors(pld_pos_err, pld_pos_err_rates,
                                          pld_vel_err, pld_vel_sp);

    tracker.computeControlOutput(dt);

    const Eigen::Vector3d result = tracker.thrust_sp();
    const Eigen::Vector3d expected = dataset.at("thrust_cmd").col(i);
    ASSERT_TRUE(result.isApprox(expected, 1e-8))
        << "Comparison failed on iteration: " << i << " where time is "
        << tout[i] << " \n";
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}