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

#ifndef LIDAR_BASE__LIDAR_BASE_HPP_
#define LIDAR_BASE__LIDAR_BASE_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <memory>
#include <functional>

namespace cyberdog
{
namespace sensor
{
class LidarBase
{
public:
  virtual bool Init(bool simulator = false) = 0;
  std::function<bool()> Open, Start, Stop, Close;
  virtual ~LidarBase() {}
  void SetPayloadCallback(
    std::function<void(
      std::shared_ptr<sensor_msgs::msg::LaserScan> payload)> cb)
  {
    payload_callback_ = cb;
  }

public:
  virtual bool Open_() = 0;
  virtual bool Start_() = 0;
  virtual bool Stop_() = 0;
  virtual bool Close_() = 0;
  virtual bool SelfCheck() = 0;
  virtual bool LowPower() = 0;
  std::function<void(std::shared_ptr<sensor_msgs::msg::LaserScan> payload)> payload_callback_;
  LidarBase() {}
};  // class LidarBase
}  // namespace sensor
}  // namespace cyberdog

#endif  // LIDAR_BASE__LIDAR_BASE_HPP_
