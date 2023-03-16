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


#ifndef ULTRASONIC_PLUGIN__ULTRASONIC_PLUGIN_HPP_
#define ULTRASONIC_PLUGIN__ULTRASONIC_PLUGIN_HPP_


#include <memory>
#include <string>
#include <map>
#include "ultrasonic_base/ultrasonic_base.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_semaphore.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_system/robot_code.hpp"

namespace cyberdog
{
namespace sensor
{
namespace EP = cyberdog::embed;
namespace SYS = cyberdog::system;

class UltrasonicCarpo : public cyberdog::sensor::UltrasonicBase
{
  using SwitchState = enum {open = 0, start, stop, close, };          // [类型]切换状态
  using Signal = cyberdog::common::Semaphore;
  using TomlParse = common::CyberdogToml;

public:
  int32_t Init(bool simulator = false) override;
  int32_t Open_() override;
  int32_t Start_() override;
  int32_t Stop_() override;
  int32_t Close_() override;
  int32_t SelfCheck() override;
  int32_t LowPowerOn() override;
  int32_t LowPowerOff() override;

public:
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
    Signal enable_on_signal;
    Signal enable_off_signal;
    Signal data_signal;
    std::atomic<bool> data_received;
    std::atomic<bool> waiting_data;
  } UltrasonicMsg;

  enum class UltrasonicCode : int32_t
  {
    kDemoError1 = 21
  };

private:
  bool simulator_;
  std::thread simulator_thread_;
  void SimulationThread();                                            // 更新模拟数据
  std::shared_ptr<SYS::CyberdogCode<UltrasonicCode>> code_{nullptr};
  std::map<std::string, std::shared_ptr<EP::Protocol<UltrasonicMsg>>> ultrasonic_map_;
  std::map<std::string, std::shared_ptr<sensor_msgs::msg::Range>> ultrasonic_data_map_;
  std::atomic<bool> is_working_;

private:
  int32_t Reset(const std::string & name);
  bool IsSingleStarted(const std::string & name);
  bool IsSingleClosed(const std::string & name);
  void UltrasonicMsgCallback(EP::DataLabel & label, std::shared_ptr<UltrasonicMsg> data);

  LOGGER_MINOR_INSTANCE("cyberdog_ultrasonic");
};  // class UltrasonicCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // ULTRASONIC_PLUGIN__ULTRASONIC_PLUGIN_HPP_
