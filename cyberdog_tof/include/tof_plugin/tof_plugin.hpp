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
#include "tof_base/tof_base.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"

#define EVM cyberdog::embed

namespace cyberdog
{
namespace sensor
{
typedef struct _tof_can
{
  uint8_t data_array[64];
  uint64_t data_clock;
  uint8_t intensity_array[64];
  uint8_t enable_on_ack;
  uint8_t enable_off_ack;
} tof_can;


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
  std::map<SwitchState, std::string> state_msg_;                      // 状态消息
  bool SingleOpen(uint8_t serial_number);
  bool SingleStop(uint8_t serial_number);
  bool SingleStart(uint8_t serial_number);
  void left_head_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);
  void right_head_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);
  void left_rear_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);
  void right_rear_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);

private:
  std::shared_ptr<protocol::msg::HeadTofPayload> head_tof_payload;
  std::shared_ptr<protocol::msg::RearTofPayload> rear_tof_payload;
  std::shared_ptr<protocol::msg::SingleTofPayload> tof_payload_left_head;
  std::shared_ptr<protocol::msg::SingleTofPayload> tof_payload_right_head;
  std::shared_ptr<protocol::msg::SingleTofPayload> tof_payload_left_rear;
  std::shared_ptr<protocol::msg::SingleTofPayload> tof_payload_right_rear;


  std::thread tof_pub_thread_simulator;
  void UpdateSimulationData();                                      // 更新模拟数据


  bool opened_;
  bool started_;
  bool closed_;
  bool stopped_;
  const int TOFOFFSET = 50;


  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_left_head;
  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_right_head;
  bool tof_opened_left_head = false;
  bool tof_started_left_head = false;
  bool tof_opened_right_head = false;
  bool tof_started_right_head = false;

  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_left_rear;
  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_right_rear;

  bool tof_opened_left_rear = false;
  bool tof_started_left_rear = false;
  bool tof_opened_right_rear = false;
  bool tof_started_right_rear = false;

  LOGGER_MINOR_INSTANCE("cyberdog_tof");
};  // class TofCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // TOF_PLUGIN__TOF_PLUGIN_HPP_
