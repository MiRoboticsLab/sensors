// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef LIDAR_PLUGIN__YDLIDAR_PLUGIN_HPP_
#define LIDAR_PLUGIN__YDLIDAR_PLUGIN_HPP_

#include <memory>
#include <thread>
#include "src/CYdLidar.h"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lidar_base/lidar_base.hpp"

namespace cyberdog
{
namespace sensor
{
class YdlidarCarpo : public cyberdog::sensor::LidarBase
{
  using ScanMsg = sensor_msgs::msg::LaserScan;                        // [topic 类型]激光数据
  using SwitchState = enum {open = 0, start, stop, close, };          // [类型]切换状态

public:
  bool Open() override;
  bool Start() override;
  bool Stop() override;
  bool Close() override;

private:
  toml::value params_toml_;                                           // 参数
  float frequency_;                                                   // 频率
  LaserScan scan_sdk;                                                 // 激光数据
  std::atomic<SwitchState> sensor_state_ {SwitchState::close};        // node 状态
  std::shared_ptr<std::thread> update_data_thread_ptr_ {nullptr};     // 更新数据线程
  std::shared_ptr<CYdLidar> lidar_ptr_ {nullptr};                     // SDK雷达对象
  std::shared_ptr<ScanMsg> scan_ptr_ {nullptr};                       // 激光数据
  void UpdateData();                                                  // 更新数据
  LOGGER_MINOR_INSTANCE("Ydlidar");
};  // class LidarCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // LIDAR_PLUGIN__YDLIDAR_PLUGIN_HPP_
