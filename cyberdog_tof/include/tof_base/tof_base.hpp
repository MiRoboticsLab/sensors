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

#ifndef TOF_BASE__TOF_BASE_HPP_
#define TOF_BASE__TOF_BASE_HPP_

#include <memory>
#include <functional>
#include "protocol/msg/single_tof_payload.hpp"
#include "protocol/msg/head_tof_payload.hpp"
#include "protocol/msg/rear_tof_payload.hpp"

namespace cyberdog
{

namespace sensor
{
class TofBase
{
public:
  virtual bool Init(bool simulator = false) = 0;
  std::function<bool()> Open, Start, Stop, Close;
  virtual bool Open_() = 0;
  virtual bool Start_() = 0;
  virtual bool Stop_() = 0;
  virtual bool Close_() = 0;
  virtual bool SelfCheck() = 0;
  virtual bool LowPower() = 0;
  virtual ~TofBase() {}
  void SetHeadPayloadCallback(
    std::function<void(
      std::shared_ptr<protocol::msg::HeadTofPayload> payload)> cb)
  {
    head_payload_callback_ = cb;
  }
  void SetRearPayloadCallback(
    std::function<void(
      std::shared_ptr<protocol::msg::RearTofPayload> payload)> cb)
  {
    rear_payload_callback_ = cb;
  }

protected:
  std::function<void(std::shared_ptr<protocol::msg::HeadTofPayload> payload)>
  head_payload_callback_;
  std::function<void(std::shared_ptr<protocol::msg::RearTofPayload> payload)>
  rear_payload_callback_;

  TofBase() {}
};  // class TofBase
}  // namespace sensor
}  // namespace cyberdog

#endif  // TOF_BASE__TOF_BASE_HPP_
