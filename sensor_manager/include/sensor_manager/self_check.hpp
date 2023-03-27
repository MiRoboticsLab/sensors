// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef SENSOR_MANAGER__SELF_CHECK_HPP_
#define SENSOR_MANAGER__SELF_CHECK_HPP_

#include <string>
#include <vector>
#include <map>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace sensor
{
class SensorSelfCheck final
{
public:
  SensorSelfCheck()
  {
    std::vector<std::string> target_vec_;
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/sensors/sensor_self_check.toml");
    toml::value config;
    if (common::CyberdogToml::ParseFile(path, config)) {
      INFO("Parse Self Check config file started, toml file is valid");
      toml::value sensors_sec;
      if (common::CyberdogToml::Get(config, "sensors", sensors_sec)) {
        INFO("Self Check init started, parse sensors config succeed");
        if (common::CyberdogToml::Get(sensors_sec, "jump", target_vec_)) {
          INFO("Self Check init started, parse jump array succeed");
        }
      }
    }
    if (target_vec_.size() > 0) {
      for (auto & elem : target_vec_) {
        if (self_check_map.find(elem) != self_check_map.end()) {
          self_check_map[elem] = true;
        }
      }
    }
  }

  bool IsJump(const std::string name)
  {
    if (self_check_map.find(name) != self_check_map.end()) {
      return self_check_map[name];
    }
    return false;
  }

private:
  std::map<std::string, bool> self_check_map = {
    {"lidar", false},
    {"gps", false},
    {"ultrasonic", false},
    {"tof", false}
  };
};
}  // namespace sensor
}  // namespace cyberdog

#endif  // SENSOR_MANAGER__SELF_CHECK_HPP_
