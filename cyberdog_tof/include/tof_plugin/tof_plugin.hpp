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
  uint8_t tof_data_array[64];
  uint64_t tof_data_clock;
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

private:
  std::map<SwitchState, std::string> state_msg_;                      // 状态消息
  bool SingleOpen(uint8_t serial_number);
  bool SingleStop(uint8_t serial_number);
  bool SingleStart(uint8_t serial_number);
  void tof_pub_callback();
  void left_front_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);
  void left_back_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);
  void right_front_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);
  void right_back_callback(std::string & name, std::shared_ptr<cyberdog::sensor::tof_can> data);

private:
  std::shared_ptr<protocol::msg::MultipleTofPayload> multiple_tof_payload;
  std::thread tof_pub_thread;
  std::thread tof_pub_thread_simulator;
  void UpdateSimulationData();                                      // 更新模拟数据


  bool opened_;
  bool started_;
  bool closed_;
  bool stopped_;


  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_left_front;
  std::shared_ptr<tof_can> tof_data_left_front;
  bool tof_opened_left_front = false;
  bool tof_started_left_front = false;

  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_left_back;
  std::shared_ptr<tof_can> tof_data_left_back;
  bool tof_opened_left_back = false;
  bool tof_started_left_back = false;

  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_right_front;
  std::shared_ptr<tof_can> tof_data_right_front;
  bool tof_opened_right_front = false;
  bool tof_started_right_front = false;

  std::shared_ptr<EVM::Protocol<tof_can>> tof_can_right_back;
  std::shared_ptr<tof_can> tof_data_right_back;
  bool tof_opened_right_back = false;
  bool tof_started_right_back = false;
  LOGGER_MINOR_INSTANCE("cyberdog_tof");
};  // class TofCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // TOF_PLUGIN__TOF_PLUGIN_HPP_
