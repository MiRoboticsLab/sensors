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

#ifndef TOF_PLUGIN__TOF_PLUGIN_HPP_
#define TOF_PLUGIN__TOF_PLUGIN_HPP_

#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <chrono>
#include "tof_base/tof_base.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_semaphore.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cyberdog
{
namespace sensor
{
namespace EP = cyberdog::embed;
namespace SYS = cyberdog::system;

class TofCarpo : public cyberdog::sensor::TofBase
{
  using SwitchState = enum {open = 0, start, stop, close, };          // [类型]切换状态
  using Signal = cyberdog::common::Semaphore;
  using Clock = std::chrono::system_clock;
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
  // 结构
  typedef struct
  {
    union {
      uint8_t data[136];
      struct
      {
        uint8_t data_array[64];
        uint64_t data_clock;
        uint8_t intensity_array[64];
      };
    };
    uint8_t enable_on_ack;
    uint8_t enable_off_ack;
    Signal enable_on_signal;
    Signal enable_off_signal;
    Signal data_signal;
    std::atomic<bool> data_received;
    std::atomic<bool> waiting_data;
    uint32_t rx_cnt;
    uint32_t rx_error_cnt;
    Clock::time_point time_start;
  } TofMsg;

  enum class TofCode : int32_t
  {
    kDemoError1 = 21
  };

private:
  // 变量
  bool simulator_;
  std::thread simulator_thread_;
  std::map<std::string, std::shared_ptr<EP::Protocol<TofMsg>>> tof_map_;
  std::map<std::string, std::shared_ptr<protocol::msg::SingleTofPayload>> tof_data_map_;
  std::atomic<bool> is_working_ = false;
  std::atomic<bool> is_low_power_ = false;
  std::shared_ptr<SYS::CyberdogCode<TofCode>> code_{nullptr};
  std::atomic<SwitchState> sensor_state_ {SwitchState::close};        // node 状态

private:
  // 方法
  void SimulationThread();                                           // 更新模拟数据
  bool IsSingleStarted(const std::string & name);
  bool IsSingleClosed(const std::string & name);
  void TofMsgCallback(EP::DataLabel & label, std::shared_ptr<TofMsg> data);
  LOGGER_MINOR_INSTANCE("cyberdog_tof");
};  // class TofCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // TOF_PLUGIN__TOF_PLUGIN_HPP_
