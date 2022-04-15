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

#ifndef ULTRASONIC_PLUGIN__ULTRASONIC_PLUGIN_HPP_
#define ULTRASONIC_PLUGIN__ULTRASONIC_PLUGIN_HPP_


#include <memory>
#include <string>
#include <map>
#include "ultrasonic_base/ultrasonic_base.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"


#define EVM cyberdog::embed

namespace cyberdog
{
namespace sensor
{
typedef struct _ultrasonic_can
{
  uint16_t ultrasonic_data;
  uint16_t ultrasonic_data_intensity;
  uint32_t ultrasonic_data_clock;
  uint8_t enable_on_ack;
  uint8_t enable_off_ack;
} ultrasonic_can;

class UltrasonicCarpo : public cyberdog::sensor::UltrasonicBase
{
  using SwitchState = enum {open = 0, start, stop, close, };          // [类型]切换状态

public:
  bool Init(bool simulator = false) override;
  bool Open_() override;
  bool Start_() override;
  bool Stop_() override;
  bool Close_() override;
  // explicit UltrasonicCarpo():cyberdog::common::CyberdogLogger(
  // "cyberdog_ultrasonic"
  // ){}

private:
  void recv_callback(std::string & name, std::shared_ptr<ultrasonic_can> data);
  void ultrasonic_pub_callback();

private:
  std::map<SwitchState, std::string> state_msg_;                      // 状态消息
  std::shared_ptr<EVM::Protocol<ultrasonic_can>> ultrasonic_can_;
  std::shared_ptr<ultrasonic_can> ultrasonic_data_;
  std::shared_ptr<sensor_msgs::msg::Range> ultrasonic_payload;
  bool opened_ = false;
  bool started_ = false;
  std::mutex mtx;
  std::thread ultrasonic_pub_thread;
  std::thread ultrasonic_pub_thread_simulator;
  void UpdateSimulationData();                                        // 更新模拟数据


  LOGGER_MINOR_INSTANCE("cyberdog_ultrasonic");
};  // class UltrasonicCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // ULTRASONIC_PLUGIN__ULTRASONIC_PLUGIN_HPP_
