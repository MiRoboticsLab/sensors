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
#include "rclcpp/rclcpp.hpp"

#define EVM cyberdog::embed

namespace cyberdog
{
namespace sensor
{
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
  cyberdog::common::Semaphore enable_on_signal;
  cyberdog::common::Semaphore enable_off_signal;
  cyberdog::common::Semaphore data_signal;
  std::atomic<bool> data_received;
  std::atomic<bool> waiting_data;
  uint32_t rx_cnt;
  uint32_t rx_error_cnt;
  std::chrono::system_clock::time_point time_start;
} TofMsg;

class TofCarpo : public cyberdog::sensor::TofBase
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
  void TofMsgCallback(EVM::DataLabel & label, std::shared_ptr<cyberdog::sensor::TofMsg> data);
  std::map<std::string, std::shared_ptr<EVM::Protocol<TofMsg>>> tof_map_;
  std::map<std::string, std::shared_ptr<protocol::msg::SingleTofPayload>> tof_data_map_;

  std::atomic<bool> is_working_;
  std::atomic<bool> opened_;
  std::atomic<bool> started_;
  std::atomic<bool> closed_;
  std::atomic<bool> stopped_;
  LOGGER_MINOR_INSTANCE("cyberdog_tof");
};  // class TofCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // TOF_PLUGIN__TOF_PLUGIN_HPP_
