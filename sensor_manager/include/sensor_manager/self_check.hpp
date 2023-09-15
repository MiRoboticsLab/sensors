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
    std::vector<std::string> target_vec;
    std::vector<std::string> critical_moudle_vec;
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/selfcheck_config.toml");
    toml::value config;
    if (common::CyberdogToml::ParseFile(path, config)) {
      INFO("Parse Self Check config file started, toml file is valid");
      toml::value sensors_sec;
      if (common::CyberdogToml::Get(config, "sensors", sensors_sec)) {
        INFO("Self Check init started, parse sensors config succeed");
        if (common::CyberdogToml::Get(sensors_sec, "jump", target_vec)) {
          INFO("Self Check init started, parse jump array succeed");
        }
        if (common::CyberdogToml::Get(sensors_sec, "critical", critical_moudle_vec)) {
          INFO("Self Check init started, parse critical array succeed");
        }
      }
    }
    if (target_vec.size() > 0) {
      for (auto & elem : target_vec) {
        if (self_check_map_.find(elem) != self_check_map_.end()) {
          self_check_map_[elem] = true;
        }
      }
    } else {
      INFO("jump sensors is empty");
    }
    if (critical_moudle_vec.size() > 0) {
      for (auto & elem : critical_moudle_vec) {
        if (critical_moudle_map_.find(elem) != critical_moudle_map_.end()) {
          critical_moudle_map_[elem] = true;
        }
      }
    } else {
      INFO("critical sensors is empty");
    }
  }

  bool IsJump(const std::string name)
  {
    if (self_check_map_.find(name) != self_check_map_.end()) {
      return self_check_map_[name];
    }
    return false;
  }

  bool IsCritical(const std::string & name)
  {
    if (critical_moudle_map_.find(name) != critical_moudle_map_.end()) {
      return critical_moudle_map_[name];
    }
    return false;
  }

private:
  std::map<std::string, bool> self_check_map_ = {
    {"lidar", false},
    {"gps", false},
    {"ultrasonic", false},
    {"tof", false}
  };
  std::map<std::string, bool> critical_moudle_map_ = {
    {"lidar", false},
    {"gps", false},
    {"ultrasonic", false},
    {"tof", false}
  };
};
}  // namespace sensor
}  // namespace cyberdog

#endif  // SENSOR_MANAGER__SELF_CHECK_HPP_
