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

#include <unistd.h>
#include <time.h>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <utility>
#include "ultrasonic_plugin/ultrasonic_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

const char * kDefaultPath = "/toml_config/sensors/";
const char * kConfigFile = "/toml_config/sensors/ultrasonic_config.toml";

int32_t cyberdog::sensor::UltrasonicCarpo::Init(bool simulator)
{
  simulator_ = simulator;
  const SYS::ModuleCode kModuleCode = SYS::ModuleCode::kUltrasonic;
  code_ = std::make_shared<SYS::CyberdogCode<UltrasonicCode>>(kModuleCode);
  this->Open = std::bind(&cyberdog::sensor::UltrasonicCarpo::Open_, this);
  this->Start = std::bind(&cyberdog::sensor::UltrasonicCarpo::Start_, this);
  this->Stop = std::bind(&cyberdog::sensor::UltrasonicCarpo::Stop_, this);
  this->Close = std::bind(&cyberdog::sensor::UltrasonicCarpo::Close_, this);

  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  std::vector<std::string> ultrasonic_cfg_files;

  auto GetCfgFile = [&]() {
      toml::value ultrasonic_config;
      auto config_file = local_share_dir + kConfigFile;
      toml::value config;
      if (!TomlParse::ParseFile(config_file, config)) {
        ERROR("toml file[%s] is invalid!", config_file.c_str());
        return false;
      }
      if (!TomlParse::Get(config, "ultrasonic_config", ultrasonic_config)) {
        ERROR("toml file[%s] get param [ultrasonic_config] failed!", config_file.c_str());
        return false;
      } else {
        if (!TomlParse::Get(ultrasonic_config, "config_files", ultrasonic_cfg_files)) {
          ERROR("toml file[%s] get param [config_files] failed!", config_file.c_str());
          return false;
        }
      }
      return true;
    };

  if (!simulator) {
    if (!GetCfgFile()) {
      ERROR("Init failed!");
      return code_->GetKeyCode(SYS::KeyCode::kFailed);
    }
    for (auto & file : ultrasonic_cfg_files) {
      auto path = local_share_dir + kDefaultPath + file;
      std::shared_ptr<EP::Protocol<UltrasonicMsg>> ultrasonic_msg =
        std::make_shared<EP::Protocol<UltrasonicMsg>>(
        path, false);
      std::shared_ptr<sensor_msgs::msg::Range> ultrasonic_data =
        std::make_shared<sensor_msgs::msg::Range>();

      if (ultrasonic_map_.find(ultrasonic_msg->GetName()) == ultrasonic_map_.end()) {
        ultrasonic_map_.insert(std::make_pair(ultrasonic_msg->GetName(), ultrasonic_msg));
        ultrasonic_data_map_.insert(std::make_pair(ultrasonic_msg->GetName(), ultrasonic_data));
        ultrasonic_msg->GetData()->data_received = false;
        ultrasonic_msg->LINK_VAR(ultrasonic_msg->GetData()->data);
        ultrasonic_msg->LINK_VAR(ultrasonic_msg->GetData()->enable_off_ack);
        ultrasonic_msg->LINK_VAR(ultrasonic_msg->GetData()->enable_on_ack);
        ultrasonic_msg->SetDataCallback(
          std::bind(
            &cyberdog::sensor::UltrasonicCarpo::
            UltrasonicMsgCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
    }
  } else {
    for (auto & file : ultrasonic_cfg_files) {
      auto path = local_share_dir + kDefaultPath + file;
      {
        toml::value single_config;
        std::string name;

        if (!common::CyberdogToml::ParseFile(path, single_config)) {
          ERROR("Init failed, toml file[%s] is invalid!", path.c_str());
          return code_->GetKeyCode(SYS::KeyCode::kFailed);
        }
        if (!common::CyberdogToml::Get(single_config, "name", name)) {
          ERROR("Init failed, toml file[%s] get param [name] failed!", path.c_str());
          return code_->GetKeyCode(SYS::KeyCode::kFailed);
        }

        ultrasonic_map_.insert(std::make_pair(name, nullptr));
      }
    }
    simulator_thread_ =
      std::thread(std::bind(&cyberdog::sensor::UltrasonicCarpo::SimulationThread, this));
    simulator_thread_.detach();
  }
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::UltrasonicCarpo::Open_()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  bool status_ok = true;

  if (!simulator_) {
    for (auto & ultrasonic : ultrasonic_map_) {
      if (ultrasonic.second->GetData()->data_received) {
        INFO("[%s] opened successfully", ultrasonic.first.c_str());
        continue;
      }
      int retry = 0;
      bool single_status_ok = true;
      while (retry++ < 3) {
        ultrasonic.second->Operate("enable_on", std::vector<uint8_t>{});
        if (ultrasonic.second->GetData()->enable_on_signal.WaitFor(1000)) {
          if (!ultrasonic.second->GetData()->data_received) {
            ERROR(
              "[%s] opened failed,can not receive enable on ack ,time[%d]",
              ultrasonic.first.c_str(), retry);
            single_status_ok = false;
          } else {
            single_status_ok = true;
            INFO("[%s] opened successfully", ultrasonic.first.c_str());
            break;
          }
        } else {
          if (ultrasonic.second->GetData()->enable_on_ack == 0) {
            if (IsSingleStarted(ultrasonic.first)) {
              INFO("[%s] opened successfully", ultrasonic.first.c_str());
              single_status_ok = true;
              break;
            } else {
              single_status_ok = false;
              WARN("[%s]opened but no data received!", ultrasonic.first.c_str());
              Reset(ultrasonic.first);
            }
          } else {
            single_status_ok = false;
            ERROR(
              "[%s] opened failed, get ack 0x%x!",
              ultrasonic.first.c_str(), ultrasonic.second->GetData()->enable_on_ack);
          }
        }
      }
      if (!single_status_ok) {status_ok = false;}
    }
  }
  if (!status_ok) {return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);}
  return return_code;
}

int32_t cyberdog::sensor::UltrasonicCarpo::Start_()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  bool status_ok = true;
  if (!simulator_) {
    for (auto & ultrasonic : ultrasonic_map_) {
      if (ultrasonic.second->GetData()->data_received) {
        INFO("[%s] started successfully", ultrasonic.first.c_str());
        continue;
      }
      if (!IsSingleStarted(ultrasonic.first)) {
        ERROR("[%s] started failed,can not receive data ", ultrasonic.first.c_str());
        status_ok = false;
      } else {
        INFO("[%s] started successfully", ultrasonic.first.c_str());
      }
    }
  }
  is_working_ = status_ok;
  if (!status_ok) {return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);}
  return return_code;
}

int32_t cyberdog::sensor::UltrasonicCarpo::Stop_()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  bool status_ok = true;
  if (!simulator_) {
    for (auto & ultrasonic : ultrasonic_map_) {
      int retry = 0;
      bool single_status_ok = true;
      while (retry++ < 3) {
        ultrasonic.second->Operate("enable_off", std::vector<uint8_t>{});
        if (ultrasonic.second->GetData()->enable_off_signal.WaitFor(1000)) {
          if (IsSingleClosed(ultrasonic.first)) {
            INFO("[%s] stoped successfully", ultrasonic.first.c_str());
            single_status_ok = true;
            break;
          }
          ERROR(
            "[%s] stoped failed,can not receive enable off ack,time[%d]",
            ultrasonic.first.c_str(), retry);
          single_status_ok = false;
        } else {
          if (ultrasonic.second->GetData()->enable_off_ack == 0) {
            // TODO(jyy) check stoped;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (IsSingleClosed(ultrasonic.first)) {
              INFO("[%s] stoped successfully", ultrasonic.first.c_str());
              single_status_ok = true;
              break;
            } else {
              ERROR(
                "[%s] stoped failed,get ack but data is receving,time[%d]",
                ultrasonic.first.c_str(), retry);
              single_status_ok = false;
            }
          } else {
            single_status_ok = false;
            ERROR(
              "[%s] stoped failed, get ack 0x%x!",
              ultrasonic.first.c_str(), ultrasonic.second->GetData()->enable_off_ack);
          }
        }
      }
      if (!single_status_ok) {status_ok = false;}
    }
  }
  is_working_ = (status_ok ? false : true);
  if (!status_ok) {return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);}
  return return_code;
}

int32_t cyberdog::sensor::UltrasonicCarpo::Close_()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  bool status_ok = true;
  if (!simulator_) {
    for (auto & ultrasonic : ultrasonic_map_) {
      if (IsSingleClosed(ultrasonic.first)) {
        INFO("[%s] closed successfully", ultrasonic.first.c_str());
      } else {
        status_ok = false;
        INFO("[%s] closed failed", ultrasonic.first.c_str());
      }
    }
  }
  is_working_ = (status_ok ? false : true);
  if (!status_ok) {return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);}
  return return_code;
}
int32_t cyberdog::sensor::UltrasonicCarpo::Reset(const std::string & name)
{
  if (ultrasonic_map_.find(name) == ultrasonic_map_.end()) {
    INFO("%s:ultrasonic map not find [%s]", __func__, name.c_str());
    return false;
  }
  INFO("%s[%s]", __func__, name.c_str());
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  bool status_ok = true;

  if (!simulator_) {
    int retry = 0;
    bool single_status_ok = true;
    while (retry++ < 3) {
      ultrasonic_map_.at(name)->Operate("enable_off", std::vector<uint8_t>{});
      if (!ultrasonic_map_.at(name)->GetData()->enable_off_signal.WaitFor(500)) {
        if (ultrasonic_map_.at(name)->GetData()->enable_off_ack == 0) {
          single_status_ok = true;
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else {
          single_status_ok = false;
          ERROR(
            "[%s] reset failed, get ack 0x%x!",
            name.c_str(), ultrasonic_map_.at(name)->GetData()->enable_off_ack);
        }
      } else {
        ERROR("[%s] reset failed, no ack![%d]", name.c_str(), retry);
      }
    }
    if (!single_status_ok) {status_ok = false;}
  }
  if (!status_ok) {return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);}
  return return_code;
}
int32_t cyberdog::sensor::UltrasonicCarpo::SelfCheck()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  if (Start() != return_code) {
    return_code = code_->GetKeyCode(SYS::KeyCode::kSelfCheckFailed);
  }
  return return_code;
}

int32_t cyberdog::sensor::UltrasonicCarpo::LowPowerOn()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  if (Stop() != return_code) {
    return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  return return_code;
}

int32_t cyberdog::sensor::UltrasonicCarpo::LowPowerOff()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  if (Open() != return_code) {
    return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  return return_code;
}

bool cyberdog::sensor::UltrasonicCarpo::IsSingleStarted(const std::string & name)
{
  if (ultrasonic_map_.find(name) == ultrasonic_map_.end()) {
    INFO("ultrasonic map not find [%s]", name.c_str());
    return false;
  }
  ultrasonic_map_.at(name)->GetData()->waiting_data = true;
  bool is_started = ultrasonic_map_.at(name)->GetData()->data_signal.WaitFor(200) ? false : true;
  ultrasonic_map_.at(name)->GetData()->waiting_data = false;
  return is_started;
}

bool cyberdog::sensor::UltrasonicCarpo::IsSingleClosed(const std::string & name)
{
  if (ultrasonic_map_.find(name) == ultrasonic_map_.end()) {
    INFO("ultrasonic map not find [%s]", name.c_str());
    return false;
  }
  ultrasonic_map_.at(name)->GetData()->waiting_data = true;
  bool is_closed = ultrasonic_map_.at(name)->GetData()->data_signal.WaitFor(200) ? true : false;
  ultrasonic_map_.at(name)->GetData()->waiting_data = false;
  ultrasonic_map_.at(name)->GetData()->data_received = false;
  return is_closed;
}

void cyberdog::sensor::UltrasonicCarpo::UltrasonicMsgCallback(
  EP::DataLabel & label,
  std::shared_ptr<UltrasonicMsg> data)
{
  if (ultrasonic_map_.find(label.group_name) != ultrasonic_map_.end()) {
    if (label.name == "enable_on_ack") {
      if (data->enable_on_ack != 0) {
        ERROR("%s,enable_on ack err 0x:%x", label.group_name.c_str(), data->enable_on_ack);
      }
      data->enable_on_signal.Give();
    } else if (label.name == "enable_off_ack") {
      if (data->enable_off_ack != 0) {
        ERROR("%s,enable_off ack err 0x:%x", label.group_name.c_str(), data->enable_off_ack);
      }
      data->enable_off_signal.Give();
    } else if (label.name == "data") {
      if (data->waiting_data) {
        data->data_signal.Give();
      }
      if (!data->data_received) {
        data->data_received = true;
      }
      struct timespec time_stu;
      clock_gettime(CLOCK_REALTIME, &time_stu);
      if (ultrasonic_data_map_.find(label.group_name) == ultrasonic_data_map_.end()) {
        ERROR("data map no msg name %s", label.group_name.c_str());
      } else {
        ultrasonic_data_map_.at(label.group_name)->header.frame_id = label.group_name;
        ultrasonic_data_map_.at(label.group_name)->header.stamp.nanosec = time_stu.tv_nsec;
        ultrasonic_data_map_.at(label.group_name)->header.stamp.sec = time_stu.tv_sec;
        ultrasonic_data_map_.at(label.group_name)->radiation_type =
          sensor_msgs::msg::Range::ULTRASOUND;
        ultrasonic_data_map_.at(label.group_name)->min_range = 0.1f;
        ultrasonic_data_map_.at(label.group_name)->max_range = 1.0f;
        ultrasonic_data_map_.at(label.group_name)->field_of_view = 15.0f;
        ultrasonic_data_map_.at(label.group_name)->range = data->ultrasonic_data * 0.001f;
        if (single_payload_callback_ != nullptr) {
          single_payload_callback_(ultrasonic_data_map_.at(label.group_name));
        }
      }
    } else {
      WARN("unknown msg name %s", label.name.c_str());
    }
  } else {
    ERROR("can drive error,error name %s", label.name.c_str());
  }
}

void cyberdog::sensor::UltrasonicCarpo::SimulationThread()
{
  while (1) {
    if (!rclcpp::ok()) {
      WARN("[cyberdog_ultrasonic]: !rclcpp::ok()");
      break;
    }
    if (is_working_) {
      std::this_thread::sleep_for(std::chrono::microseconds(200000));
      auto ultrasonic_payload = std::make_shared<sensor_msgs::msg::Range>();
      struct timespec time_stu;
      clock_gettime(CLOCK_REALTIME, &time_stu);
      ultrasonic_payload->header.stamp.nanosec = time_stu.tv_nsec;
      ultrasonic_payload->header.stamp.sec = time_stu.tv_sec;
      ultrasonic_payload->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
      ultrasonic_payload->min_range = 0.1f;
      ultrasonic_payload->max_range = 1.0f;
      ultrasonic_payload->field_of_view = 15.0f;
      ultrasonic_payload->range = 0.001f;

      for (auto & ultrasonic : ultrasonic_map_) {
        ultrasonic_payload->header.frame_id = ultrasonic.first;
        if (single_payload_callback_ != nullptr) {
          single_payload_callback_(ultrasonic_payload);
        }
      }
    }
  }
}
PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::UltrasonicCarpo, cyberdog::sensor::UltrasonicBase)
