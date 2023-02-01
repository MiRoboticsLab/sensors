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
#include "cyberdog_common/cyberdog_semaphore.hpp"


#define EVM cyberdog::embed

namespace cyberdog
{
namespace sensor
{
typedef struct
{
  union {
    uint8_t data[8];
    struct
    {
      uint16_t ultrasonic_data;
      uint16_t ultrasonic_data_intensity;
      uint32_t ultrasonic_data_clock;
    };
  };
  uint8_t enable_on_ack;
  uint8_t enable_off_ack;
  cyberdog::common::Semaphore enable_on_signal;
  cyberdog::common::Semaphore enable_off_signal;
  cyberdog::common::Semaphore data_signal;
  std::atomic<bool> data_received;
  std::atomic<bool> waiting_data;
  std::chrono::system_clock::time_point time_start;
} UltrasonicMsg;

class UltrasonicCarpo : public cyberdog::sensor::UltrasonicBase
{
  using SwitchState = enum {open = 0, start, stop, close, };          // [类型]切换状态

public:
  bool Init(bool simulator = false) override;
  bool Open_() override;
  bool Start_() override;
  bool Stop_() override;
  bool Close_() override;
  bool SelfCheck() override;
  bool LowPower() override;

private:
  bool simulator_;
  std::thread simulator_thread_;
  void SimulationThread();                                            // 更新模拟数据
  std::map<SwitchState, std::string> state_msg_;                      // 状态消息

private:
  bool IsSingleStarted(const std::string & name);
  bool IsSingleClosed(const std::string & name);
  void UltrasonicMsgCallback(EVM::DataLabel & label, std::shared_ptr<UltrasonicMsg> data);
  std::map<std::string, std::shared_ptr<EVM::Protocol<UltrasonicMsg>>> ultrasonic_map_;
  std::map<std::string, std::shared_ptr<sensor_msgs::msg::Range>> ultrasonic_data_map_;

  std::atomic<bool> is_working_;
  std::atomic<bool> opened_ = false;
  std::atomic<bool> started_ = false;
  LOGGER_MINOR_INSTANCE("cyberdog_ultrasonic");
};  // class UltrasonicCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // ULTRASONIC_PLUGIN__ULTRASONIC_PLUGIN_HPP_
