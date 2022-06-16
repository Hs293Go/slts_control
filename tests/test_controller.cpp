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

class TestController : public ::testing::Test {
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
      dataset_.emplace(key, Eigen::MatrixXd(sz[0].asInt(), sz[1].asInt()));
      std::transform(val.begin(), val.end(), dataset_[key].data(),
                     std::mem_fn(&Json::Value::asDouble));
    }
  }

  std::unordered_map<std::string, Eigen::MatrixXd> dataset_;
};

int main(int argc, char** argv) {}