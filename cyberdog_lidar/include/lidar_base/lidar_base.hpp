// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
/*! \file       lidar_base.hpp
    \brief      雷达基础模块。
    \details    创建及初始化雷达基础模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class LidarBase
{
public:
  virtual int32_t Init(bool simulator = false) = 0;
  std::function<int32_t()> Open, Start, Stop, Close;
  virtual ~LidarBase() {}
  void SetPayloadCallback(
    std::function<void(
      std::shared_ptr<sensor_msgs::msg::LaserScan> payload)> cb)
  {
    payload_callback_ = cb;
  }

public:
  virtual int32_t Open_() = 0;
  virtual int32_t Start_() = 0;
  virtual int32_t Stop_() = 0;
  virtual int32_t Close_() = 0;
  virtual int32_t SelfCheck() = 0;
  virtual int32_t LowPowerOn() = 0;
  virtual int32_t LowPowerOff() = 0;
  std::function<void(std::shared_ptr<sensor_msgs::msg::LaserScan> payload)> payload_callback_;
  LidarBase() {}
};  // class LidarBase
}  // namespace sensor
}  // namespace cyberdog

#endif  // LIDAR_BASE__LIDAR_BASE_HPP_
