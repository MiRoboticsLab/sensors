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

void cyberdog::sensor::GpsCarpo::Open()
{
  bcmgps_ = std::make_shared<bcm_gps::GPS>();
  bcmgps_->SetCallback(
    std::bind(
      &GpsCarpo::BCMGPS_Payload_callback, this,
      std::placeholders::_1));
}

void cyberdog::sensor::GpsCarpo::Start()
{
  if (bcmgps_ != nullptr) {bcmgps_->Start();}
}

void cyberdog::sensor::GpsCarpo::Stop()
{
  if (bcmgps_ != nullptr) {bcmgps_->Stop();}
}

void cyberdog::sensor::GpsCarpo::Close()
{
  if (bcmgps_ != nullptr) {bcmgps_->Close();}
  bcmgps_ = nullptr;
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
  if (payload_callback_ != nullptr) {payload_callback_(cyberdog_payload);}
}

PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::GpsCarpo, cyberdog::sensor::GpsBase)
