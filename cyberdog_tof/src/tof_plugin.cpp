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

#include <unistd.h>
#include <time.h>
#include <memory>
#include <ctime>
#include <vector>
#include <string>
#include <utility>
#include "tof_plugin/tof_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"

const int kTofOffset = 50;
const char * kDefaultPath = "/toml_config/sensors/";
const char * kConfigFile = "/toml_config/sensors/tof_config.toml";
const int kMsgCheckInterval = 60000;  // ms

bool cyberdog::sensor::TofCarpo::Init(bool simulator)
{
  simulator_ = simulator;
  this->state_msg_.insert({SwitchState::open, "Open"});
  this->state_msg_.insert({SwitchState::start, "Start"});
  this->state_msg_.insert({SwitchState::stop, "Stop"});
  this->state_msg_.insert({SwitchState::close, "Close"});
  this->Open = std::bind(&cyberdog::sensor::TofCarpo::Open_, this);
  this->Start = std::bind(&cyberdog::sensor::TofCarpo::Start_, this);
  this->Stop = std::bind(&cyberdog::sensor::TofCarpo::Stop_, this);
  this->Close = std::bind(&cyberdog::sensor::TofCarpo::Close_, this);

  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto config_file = local_share_dir + kConfigFile;
  toml::value config;
  if (!common::CyberdogToml::ParseFile(config_file, config)) {
    ERROR("Init failed, toml file[%s] is invalid!", config_file.c_str());
    return false;
  }

  toml::value tof_config;
  std::vector<std::string> tof_cfg_files;

  if (!common::CyberdogToml::Get(config, "tof_config", tof_config)) {
    ERROR("Init failed, toml file[%s] get param [tof_config] failed!", config_file.c_str());
    return false;
  } else {
    if (!common::CyberdogToml::Get(tof_config, "config_files", tof_cfg_files)) {
      ERROR("Init failed, toml file[%s] get param [config_files] failed!", config_file.c_str());
      return false;
    }
  }
  if (!simulator_) {
    for (auto & file : tof_cfg_files) {
      auto path = local_share_dir + kDefaultPath + file;
      std::shared_ptr<EVM::Protocol<TofMsg>> tof_msg = std::make_shared<EVM::Protocol<TofMsg>>(
        path, false);

      std::shared_ptr<protocol::msg::SingleTofPayload> tof_data =
        std::make_shared<protocol::msg::SingleTofPayload>();
      if (tof_map_.find(tof_msg->GetName()) == tof_map_.end()) {
        tof_map_.insert(std::make_pair(tof_msg->GetName(), tof_msg));
        tof_data_map_.insert(std::make_pair(tof_msg->GetName(), tof_data));
        tof_msg->GetData()->data_received = false;
        tof_msg->LINK_VAR(tof_msg->GetData()->data);
        tof_msg->LINK_VAR(tof_msg->GetData()->enable_off_ack);
        tof_msg->LINK_VAR(tof_msg->GetData()->enable_on_ack);
        tof_msg->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            TofMsgCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
    }
  } else {
    for (auto & file : tof_cfg_files) {
      auto path = local_share_dir + kDefaultPath + file;
      {
        toml::value single_config;
        std::string name;

        if (!common::CyberdogToml::ParseFile(path, single_config)) {
          ERROR("Init failed, toml file[%s] is invalid!", path.c_str());
          return false;
        }
        if (!common::CyberdogToml::Get(single_config, "name", name)) {
          ERROR("Init failed, toml file[%s] get param [name] failed!", path.c_str());
          return false;
        }

        tof_map_.insert(std::make_pair(name, nullptr));
      }
    }
    simulator_thread_ = std::thread(std::bind(&cyberdog::sensor::TofCarpo::SimulationThread, this));
    simulator_thread_.detach();
  }
  return true;
}


bool cyberdog::sensor::TofCarpo::Open_()
{
  bool status_ok = true;

  if (!simulator_) {
    for (auto & tof : tof_map_) {
      tof.second->GetData()->time_start = std::chrono::system_clock::now();
      tof.second->GetData()->rx_cnt = 0;
      tof.second->GetData()->rx_error_cnt = 0;
      if (tof.second->GetData()->data_received) {
        INFO("[%s] opened successfully", tof.first.c_str());
        continue;
      }
      int retry = 0;
      bool single_status_ok = true;
      while (retry++ < 3) {
        tof.second->Operate("enable_on", std::vector<uint8_t>{});
        if (tof.second->GetData()->enable_on_signal.WaitFor(1000)) {
          if (!tof.second->GetData()->data_received) {
            ERROR(
              "[%s] opened failed,can not receive enable on ack ,time[%d]",
              tof.first.c_str(), retry);
            single_status_ok = false;
          } else {
            single_status_ok = true;
            INFO("[%s] opened successfully", tof.first.c_str());
            break;
          }
        } else {
          if (tof.second->GetData()->enable_on_ack == 0) {
            single_status_ok = true;
            INFO("[%s] opened successfully", tof.first.c_str());
            break;
          } else {
            single_status_ok = false;
            ERROR(
              "[%s] opened failed, get ack 0x%x!", tof.first.c_str(),
              tof.second->GetData()->enable_on_ack);
          }
        }
      }
      if (!single_status_ok) {status_ok = false;}
    }
  }
  return status_ok;
}


bool cyberdog::sensor::TofCarpo::Start_()
{
  bool status_ok = true;
  if (!simulator_) {
    for (auto & tof : tof_map_) {
      if (tof.second->GetData()->data_received) {
        INFO("[%s] started successfully", tof.first.c_str());
        continue;
      }
      if (!IsSingleStarted(tof.first)) {
        ERROR("[%s] started failed,can not receive data ", tof.first.c_str());
        status_ok = false;
      } else {
        INFO("[%s] started successfully", tof.first.c_str());
      }
    }
  }
  is_working_ = status_ok;
  return status_ok;
}

bool cyberdog::sensor::TofCarpo::Stop_()
{
  bool status_ok = true;
  if (!simulator_) {
    for (auto & tof : tof_map_) {
      int retry = 0;
      bool single_status_ok = true;
      while (retry++ < 3) {
        tof.second->Operate("enable_off", std::vector<uint8_t>{});
        if (tof.second->GetData()->enable_off_signal.WaitFor(1000)) {
          if (IsSingleClosed(tof.first)) {
            INFO("[%s] stoped successfully", tof.first.c_str());
            single_status_ok = true;
            break;
          }
          ERROR(
            "[%s] stoped failed,can not receive enable off ack,time[%d]",
            tof.first.c_str(), retry);
          single_status_ok = false;
        } else {
          if (tof.second->GetData()->enable_off_ack == 0) {
            INFO("[%s] stoped successfully", tof.first.c_str());
            single_status_ok = true;
            break;
          } else {
            single_status_ok = false;
            ERROR(
              "[%s] stoped failed, get ack 0x%x!", tof.first.c_str(),
              tof.second->GetData()->enable_off_ack);
          }
        }
      }
      if (!single_status_ok) {status_ok = false;}
    }
  }
  is_working_ = (status_ok ? false : true);
  return status_ok;
}

bool cyberdog::sensor::TofCarpo::Close_()
{
  bool status_ok = true;
  if (!simulator_) {
    for (auto & tof : tof_map_) {
      if (IsSingleClosed(tof.first)) {
        INFO("[%s] closed successfully", tof.first.c_str());
      } else {
        status_ok = false;
        INFO("[%s] closed failed", tof.first.c_str());
      }
    }
  }
  is_working_ = (status_ok ? false : true);
  return status_ok;
}

bool cyberdog::sensor::TofCarpo::SelfCheck()
{
  return is_working_;
}

bool cyberdog::sensor::TofCarpo::LowPower()
{
  return true;
}

void cyberdog::sensor::TofCarpo::SimulationThread()
{
  while (is_working_) {
    if (!rclcpp::ok()) {
      WARN("[cyberdog_tof]: !rclcpp::ok()");
      break;
    }
    INFO("[cyberdog_tof]: publish cyberdog_tof payload succeed");
    std::this_thread::sleep_for(std::chrono::microseconds(200000));
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj;
    for (size_t i = 0; i < datanum; i++) {
      obj.push_back(1.0f * protocol::msg::SingleTofPayload::SCALE_FACTOR);
    }
    auto tof_payload = std::make_shared<protocol::msg::SingleTofPayload>();
    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload->header.stamp.sec = time_stu.tv_sec;
    tof_payload->data = obj;
    tof_payload->data_available = false;

    for (auto & tof : tof_map_) {
      tof_payload->header.frame_id = tof.first;
      if (single_payload_callback_ != nullptr) {
        single_payload_callback_(tof_payload);
      }
    }
    INFO("[cyberdog_tof]: publish cyberdog_tof payload succeed");
  }
}
bool cyberdog::sensor::TofCarpo::IsSingleStarted(const std::string & name)
{
  if (tof_map_.find(name) == tof_map_.end()) {
    INFO("tof map not find [%s]", name.c_str());
    return false;
  }
  tof_map_.at(name)->GetData()->waiting_data = true;
  bool is_started = tof_map_.at(name)->GetData()->data_signal.WaitFor(2000) ? false : true;
  tof_map_.at(name)->GetData()->waiting_data = false;
  return is_started;
}

bool cyberdog::sensor::TofCarpo::IsSingleClosed(const std::string & name)
{
  if (tof_map_.find(name) == tof_map_.end()) {
    INFO("tof map not find [%s]", name.c_str());
    return false;
  }
  tof_map_.at(name)->GetData()->waiting_data = true;
  bool is_closed = tof_map_.at(name)->GetData()->data_signal.WaitFor(2000) ? true : false;
  tof_map_.at(name)->GetData()->waiting_data = false;
  tof_map_.at(name)->GetData()->data_received = false;
  return is_closed;
}

void cyberdog::sensor::TofCarpo::TofMsgCallback(
  EVM::DataLabel & label,
  std::shared_ptr<cyberdog::sensor::TofMsg> data)
{
  if (tof_map_.find(label.group_name) != tof_map_.end()) {
    if (label.name == "enable_on_ack") {
      if (data->enable_on_ack != 0) {
        ERROR("%s,enable_on ack err 0x:%x", label.group_name.c_str(), data->enable_on_ack);
      }
      tof_map_.at(label.group_name)->GetData()->enable_on_signal.Give();
    } else if (label.name == "enable_off_ack") {
      if (data->enable_off_ack != 0) {
        ERROR("%s,enable_off ack err 0x:%x", label.group_name.c_str(), data->enable_off_ack);
      }
      tof_map_.at(label.group_name)->GetData()->enable_off_signal.Give();
    } else if (label.name == "data") {
      if (tof_map_.at(label.group_name)->GetData()->waiting_data) {
        tof_map_.at(label.group_name)->GetData()->data_signal.Give();
      }
      if (!tof_map_.at(label.group_name)->GetData()->data_received) {
        tof_map_.at(label.group_name)->GetData()->data_received = true;
      }
      const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
      std::vector<float> obj_data;
      std::vector<float> obj_intensity;
      for (size_t i = 0; i < datanum; i++) {
        obj_data.push_back(
          (data->data_array[i] * 2.0f + kTofOffset) *
          protocol::msg::SingleTofPayload::SCALE_FACTOR);
        obj_intensity.push_back(
          data->intensity_array[i]);
      }

      struct timespec time_stu;
      clock_gettime(CLOCK_REALTIME, &time_stu);
      if (tof_data_map_.find(label.group_name) == tof_data_map_.end()) {
        ERROR("data map no msg name %s", label.group_name.c_str());
      } else {
        tof_map_.at(label.group_name)->GetData()->rx_cnt++;
        if (!label.is_full) {
          tof_map_.at(label.group_name)->GetData()->rx_error_cnt++;
        }

        // msg check
        auto now = std::chrono::system_clock::now();
        auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          now - tof_map_.at(label.group_name)->GetData()->time_start);
        if (duration.count() >= kMsgCheckInterval) {
          tof_map_.at(label.group_name)->GetData()->time_start = std::chrono::system_clock::now();
          if (tof_map_.at(label.group_name)->GetData()->rx_error_cnt > 0) {
            WARN(
              "[%s] get error data:[%d]/[%d] ,interval[%d] !",
              label.group_name.c_str(), tof_map_.at(
                label.group_name)->GetData()->rx_error_cnt, tof_map_.at(
                label.group_name)->GetData()->rx_cnt, kMsgCheckInterval);
            tof_map_.at(label.group_name)->GetData()->rx_error_cnt = 0;
          }
          tof_map_.at(label.group_name)->GetData()->rx_cnt = 0;
        }

        tof_data_map_.at(label.group_name)->header.frame_id = label.group_name;
        tof_data_map_.at(label.group_name)->header.stamp.nanosec = time_stu.tv_nsec;
        tof_data_map_.at(label.group_name)->header.stamp.sec = time_stu.tv_sec;
        tof_data_map_.at(label.group_name)->data = obj_data;
        tof_data_map_.at(label.group_name)->intensity = obj_intensity;
        tof_data_map_.at(label.group_name)->data_available = label.is_full;
        if (single_payload_callback_ != nullptr) {
          single_payload_callback_(tof_data_map_.at(label.group_name));
        }
      }
    } else {
      WARN("unknown msg name %s", label.name.c_str());
    }
  } else {
    ERROR("can drive error,error name %s", label.name.c_str());
  }
}
PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::TofCarpo, cyberdog::sensor::TofBase)
