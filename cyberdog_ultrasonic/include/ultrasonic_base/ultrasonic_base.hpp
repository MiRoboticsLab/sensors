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

#ifndef ULTRASONIC_BASE__ULTRASONIC_BASE_HPP_
#define ULTRASONIC_BASE__ULTRASONIC_BASE_HPP_

#include <sensor_msgs/msg/range.hpp>
#include <memory>
#include <functional>

namespace cyberdog
{

namespace sensor
{
class UltrasonicBase
{
public:
  virtual int32_t Init(bool simulator = false) = 0;
  std::function<int32_t()> Open, Start, Stop, Close;
  virtual int32_t Open_() = 0;
  virtual int32_t Start_() = 0;
  virtual int32_t Stop_() = 0;
  virtual int32_t Close_() = 0;
  virtual int32_t SelfCheck() = 0;
  virtual int32_t LowPowerOn() = 0;
  virtual int32_t LowPowerOff() = 0;
  virtual ~UltrasonicBase() {}
  void SetSinglePayloadCallback(
    std::function<void(
      std::shared_ptr<sensor_msgs::msg::Range> payload)> cb)
  {
    single_payload_callback_ = cb;
  }

protected:
  std::function<void(std::shared_ptr<sensor_msgs::msg::Range> payload)> single_payload_callback_;
  // protocol::msg::UltrasonicPayload
  UltrasonicBase() {}
};  // class UltrasonicBase
}  // namespace sensor
}  // namespace cyberdog

#endif  // ULTRASONIC_BASE__ULTRASONIC_BASE_HPP_
