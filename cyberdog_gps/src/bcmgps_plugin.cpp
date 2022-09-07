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

#include <memory>

#include "bcmgps_plugin/bcmgps_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"


bool cyberdog::sensor::GpsCarpo::Init(bool simulator)
{
  this->state_msg_.insert({SwitchState::open, "Open"});
  this->state_msg_.insert({SwitchState::start, "Start"});
  this->state_msg_.insert({SwitchState::stop, "Stop"});
  this->state_msg_.insert({SwitchState::close, "Close"});

  if (!simulator) {
    this->Open = std::bind(&cyberdog::sensor::GpsCarpo::Open_, this);
    this->Start = std::bind(&cyberdog::sensor::GpsCarpo::Start_, this);
    this->Stop = std::bind(&cyberdog::sensor::GpsCarpo::Stop_, this);
    this->Close = std::bind(&cyberdog::sensor::GpsCarpo::Close_, this);
  } else {
    auto Simulator = [this](SwitchState now_state) -> bool {
        INFO("%s gps ...", this->state_msg_[now_state].c_str());
        switch (now_state) {
          case SwitchState::open:
          case SwitchState::stop:
            break;
          case SwitchState::start:
            gps_pub_thread_simulator =
              std::thread(std::bind(&cyberdog::sensor::GpsCarpo::UpdateSimulationData, this));
            break;
          case SwitchState::close:
            if ((&gps_pub_thread_simulator != nullptr) &&
              gps_pub_thread_simulator.joinable())
            {
              gps_pub_thread_simulator.join();
            }
            break;
          default:
            WARN("gps not recognized state");
            break;
        }

        INFO("gps %s ok", this->state_msg_[now_state].c_str());
        return true;
      };
    this->Open = std::bind(Simulator, SwitchState::open);
    this->Start = std::bind(Simulator, SwitchState::start);
    this->Stop = std::bind(Simulator, SwitchState::stop);
    this->Close = std::bind(Simulator, SwitchState::close);
  }
  return true;
}


bool cyberdog::sensor::GpsCarpo::Open_()
{
  bcmgps_ = std::make_shared<bcm_gps::GPS>();
  bcmgps_->SetCallback(
    std::bind(
      &GpsCarpo::BCMGPS_Payload_callback, this,
      std::placeholders::_1));
  bool is_open = bcmgps_->IsOpened();
  if (is_open == true) {
    INFO("[cyberdog_gps] gps open successfully");
  } else {
    INFO("[cyberdog_gps] gps open failed");
  }
  return is_open;
}

bool cyberdog::sensor::GpsCarpo::Start_()
{
  if (bcmgps_ != nullptr) {bcmgps_->Start();}
  bool is_start = bcmgps_->IsStarted();
  if (is_start == true) {
    INFO("[cyberdog_gps] gps start successfully");
  } else {
    INFO("[cyberdog_gps] gps start failed");
  }

  return is_start;
}

bool cyberdog::sensor::GpsCarpo::Stop_()
{
  if (bcmgps_ != nullptr) {bcmgps_->Stop();}
  bool is_stop = !(bcmgps_->IsStarted());
  if (is_stop == true) {
    INFO("[cyberdog_gps] gps stop successfully");
  } else {
    INFO("[cyberdog_gps] gps stop failed");
  }
  return is_stop;
}

bool cyberdog::sensor::GpsCarpo::Close_()
{
  if (bcmgps_ != nullptr) {bcmgps_->Close();}
  bool is_close = !(bcmgps_->IsOpened());
  if (is_close == true) {
    INFO("[cyberdog_gps] gps close successfully");
  } else {
    INFO("[cyberdog_gps] gps close failed");
  }
  return is_close;
}

void cyberdog::sensor::GpsCarpo::BCMGPS_Payload_callback(
  std::shared_ptr<GPS_Payload> payload)
{
  // auto cyberdog_payload = std::make_shared<cyberdog::sensor::Cyberdog_GPS_payload>();
  auto cyberdog_payload = std::make_shared<protocol::msg::GpsPayload>();
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  cyberdog_payload->sec = time_stu.tv_sec;
  cyberdog_payload->nanosec = time_stu.tv_nsec;
  cyberdog_payload->itow = payload->iTOW;
  cyberdog_payload->lat = payload->lat * 1e-7;
  cyberdog_payload->lon = payload->lon * 1e-7;
  cyberdog_payload->fix_type = payload->fixType;
  cyberdog_payload->num_sv = payload->numSV;
  if (payload_callback_ != nullptr) {payload_callback_(cyberdog_payload);} else {
    INFO("[cyberdog_gps]: payload_callback_==nullptr ");
  }
}
void cyberdog::sensor::GpsCarpo::UpdateSimulationData()
{
  auto cyberdog_payload = std::make_shared<protocol::msg::GpsPayload>();
  while (true) {
    if (!rclcpp::ok()) {
      WARN("[cyberdog_gps]: !rclcpp::ok()");
      break;
    }
    // INFO("[cyberdog_gps]: publish gps payload succeed");
    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    cyberdog_payload->sec = time_stu.tv_sec;
    cyberdog_payload->nanosec = time_stu.tv_nsec;
    cyberdog_payload->itow = static_cast<uint32_t>(0);
    cyberdog_payload->lat = static_cast<_Float64>(0);
    cyberdog_payload->lon = static_cast<_Float64>(0);
    cyberdog_payload->fix_type = static_cast<uint8_t>(0);
    cyberdog_payload->num_sv = static_cast<uint8_t>(0);
    payload_callback_(cyberdog_payload);
    // INFO("[cyberdog_gps]: publish gps payload succeed");
  }
}
PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::GpsCarpo, cyberdog::sensor::GpsBase)
